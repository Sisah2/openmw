#include "navmeshdb.hpp"

#include <components/debug/debuglog.hpp>

#include <lz4.h>

#include <DetourAlloc.h>

#include <sqlite3.h>

#include <cstddef>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace DetourNavigator
{
    namespace
    {
        constexpr char schema[] = R"(
            BEGIN TRANSACTION;

            CREATE TABLE IF NOT EXISTS tiles (
                tile_id INTEGER PRIMARY KEY,
                revision INTEGER NOT NULL DEFAULT 1,
                worldspace TEXT NOT NULL,
                tile_position_x INTEGER NOT NULL,
                tile_position_y INTEGER NOT NULL,
                version INTEGER NOT NULL,
                input BLOB,
                data BLOB
            );

            CREATE UNIQUE INDEX IF NOT EXISTS index_unique_tiles_by_worldspace_and_tile_position_and_input
                ON tiles (worldspace, tile_position_x, tile_position_y, input);

            COMMIT;
        )";

        constexpr std::string_view getMaxTileIdQuery = R"(
            SELECT max(tile_id) FROM tiles
        )";

        constexpr std::string_view findTileQuery = R"(
            SELECT tile_id, version
            FROM tiles
            WHERE worldspace = :worldspace
            AND tile_position_x = :tile_position_x
            AND tile_position_y = :tile_position_y
            AND input = :input
        )";

        constexpr std::string_view getTileDataQuery = R"(
            SELECT tile_id, version, data
            FROM tiles
            WHERE worldspace = :worldspace
            AND tile_position_x = :tile_position_x
            AND tile_position_y = :tile_position_y
            AND input = :input
        )";

        constexpr std::string_view insertTileQuery = R"(
            INSERT INTO tiles ( tile_id,  worldspace,  version,  tile_position_x,  tile_position_y,  input,  data)
                    VALUES (:tile_id, :worldspace, :version, :tile_position_x, :tile_position_y, :input, :data)
        )";

        constexpr std::string_view updateTileQuery = R"(
            UPDATE tiles
            SET version = :version,
                data = :data,
                revision = revision + 1
            WHERE tile_id = :tile_id
        )";

        template <class T>
        void copyColumn(sqlite3& /*db*/, sqlite3_stmt& statement, int index, int type, T& value)
        {
            switch (type)
            {
                case SQLITE_INTEGER:
                    value = static_cast<T>(sqlite3_column_int64(&statement, index));
                    break;
                case SQLITE_FLOAT:
                    value = static_cast<T>(sqlite3_column_double(&statement, index));
                    break;
            }
        }

        void copyColumn(sqlite3& db, sqlite3_stmt& statement, int index, int type, std::vector<std::byte>& value)
        {
            if (type != SQLITE_BLOB)
                throw std::logic_error("Type of column " + std::to_string(index) + " is " + std::to_string(type)
                                       + " that does not match expected output type: SQLITE_BLOB");
            const void* const blob = sqlite3_column_blob(&statement, index);
            if (blob == nullptr)
            {
                if (const int ec = sqlite3_errcode(&db); ec != SQLITE_OK)
                    throw std::runtime_error("Failed to read blob from column " + std::to_string(index)
                                             + ": " + sqlite3_errmsg(&db));
                value.clear();
                return;
            }
            const int size = sqlite3_column_bytes(&statement, index);
            if (size <= 0)
            {
                if (const int ec = sqlite3_errcode(&db); ec != SQLITE_OK)
                    throw std::runtime_error("Failed to get column bytes " + std::to_string(index)
                                             + ": " + sqlite3_errmsg(&db));
                value.clear();
                return;
            }
            value.reserve(static_cast<std::size_t>(size));
            value.assign(static_cast<const std::byte*>(blob), static_cast<const std::byte*>(blob) + size);
        }

        template <int index, class T>
        void getColumnsImpl(sqlite3& db, sqlite3_stmt& statement, T& row)
        {
            if constexpr (0 < index && index <= std::tuple_size_v<T>)
            {
                const int column = index - 1;
                if (const int number = sqlite3_column_count(&statement); column >= number)
                    throw std::out_of_range("Column number is out of range: " + std::to_string(column)
                                            + " >= " + std::to_string(number));
                const int type = sqlite3_column_type(&statement, column);
                switch (type)
                {
                    case SQLITE_INTEGER:
                    case SQLITE_FLOAT:
                    case SQLITE_BLOB:
                        copyColumn(db, statement, column, type, std::get<index - 1>(row));
                        break;
                    case SQLITE_NULL:
                        std::get<index - 1>(row) = std::decay_t<decltype(std::get<index - 1>(row))> {};
                        break;
                    case SQLITE_TEXT:
                        throw std::logic_error("Text column type is not supported");
                    default:
                        throw std::runtime_error("Column " + std::to_string(column)
                                                 + " has unnsupported column type: " + std::to_string(type));
                }
                getColumnsImpl<index - 1>(db, statement, row);
            }
        }

        template <class T>
        void getColumns(sqlite3& db, sqlite3_stmt& statement, T& row)
        {
            getColumnsImpl<std::tuple_size_v<T>>(db, statement, row);
        }

        template <class T>
        void getRow(sqlite3& db, sqlite3_stmt& statement, T& row)
        {
            auto tuple = std::tie(row);
            getColumns(db, statement, tuple);
        }

        template <class ... Args>
        void getRow(sqlite3& db, sqlite3_stmt& statement, std::tuple<Args ...>& row)
        {
            getColumns(db, statement, row);
        }

        template <class T, class ... Args>
        void prepare(sqlite3& db, SqliteStatement<T>& statement, Args&& ... args)
        {
            if (statement.mNeedReset)
            {
                if (sqlite3_reset(statement.mStatement.get()) == SQLITE_OK
                        && sqlite3_clear_bindings(statement.mStatement.get()) == SQLITE_OK)
                    statement.mNeedReset = false;
                else
                    statement.mStatement = makeStatement(db, statement.mQuery.text());
            }
            statement.mQuery.bind(db, *statement.mStatement, std::forward<Args>(args) ...);
        }

        template <class T>
        bool executeStep(sqlite3& db, const SqliteStatement<T>& statement)
        {
            switch (sqlite3_step(statement.mStatement.get()))
            {
                case SQLITE_ROW: return true;
                case SQLITE_DONE: return false;
            }
            throw std::runtime_error("Failed to execute statement step: " + std::string(sqlite3_errmsg(&db)));
        }

        template <class T, class I, class ... Args>
        I request(sqlite3& db, SqliteStatement<T>& statement, I out, std::size_t max, Args&& ... args)
        {
            try
            {
                statement.mNeedReset = true;
                prepare(db, statement, std::forward<Args>(args) ...);
                for (std::size_t i = 0; executeStep(db, statement) && i < max; ++i)
                    getRow(db, *statement.mStatement, *out++);
                return out;
            }
            catch (const std::exception& e)
            {
                throw std::runtime_error("Failed perform request \"" + std::string(statement.mQuery.text())
                                        + "\": " + std::string(e.what()));
            }
        }

        template <class T, class ... Args>
        int execute(sqlite3& db, SqliteStatement<T>& statement, Args&& ... args)
        {
            try
            {
                statement.mNeedReset = true;
                prepare(db, statement, std::forward<Args>(args) ...);
                if (executeStep(db, statement))
                    throw std::logic_error("Execute cannot return rows");
                return sqlite3_changes(&db);
            }
            catch (const std::exception& e)
            {
                throw std::runtime_error("Failed to execute statement \"" + std::string(statement.mQuery.text())
                                        + "\": " + std::string(e.what()));
            }
        }

        Sqlite3Ptr makeDb(std::string_view path)
        {
            sqlite3* handle = nullptr;
            const int flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE;
            if (const int ec = sqlite3_open_v2(std::string(path).c_str(), &handle, flags, nullptr); ec != SQLITE_OK)
            {
                const std::string message(sqlite3_errmsg(handle));
                sqlite3_close(handle);
                throw std::runtime_error("Failed to open database: " + message);
            }
            Sqlite3Ptr result(handle);
            if (const int ec = sqlite3_exec(result.get(), schema, nullptr, nullptr, nullptr); ec != SQLITE_OK)
                throw std::runtime_error("Failed create database schema: " + std::string(sqlite3_errmsg(handle)));
            return result;
        }

        void bindParameter(sqlite3& db, sqlite3_stmt& stmt, int index, int value)
        {
            if (const int ec = sqlite3_bind_int(&stmt, index, value); ec != SQLITE_OK)
                throw std::runtime_error("Failed to bind int to parameter " + std::to_string(index)
                                         + ": " + std::string(sqlite3_errmsg(&db)));
        }

        void bindParameter(sqlite3& db, sqlite3_stmt& stmt, int index, std::int64_t value)
        {
            if (const int ec = sqlite3_bind_int64(&stmt, index, value); ec != SQLITE_OK)
                throw std::runtime_error("Failed to bind int64 to parameter " + std::to_string(index)
                                         + ": " + std::string(sqlite3_errmsg(&db)));
        }

        void bindParameter(sqlite3& db, sqlite3_stmt& stmt, int index, std::string_view value)
        {
            if (sqlite3_bind_text(&stmt, index, value.data(), static_cast<int>(value.size()), SQLITE_STATIC) != SQLITE_OK)
                throw std::runtime_error("Failed to bind text to parameter " + std::to_string(index)
                                         + ": " + std::string(sqlite3_errmsg(&db)));
        }

        void bindParameter(sqlite3& db, sqlite3_stmt& stmt, int index, const std::vector<std::byte>& value)
        {
            if (sqlite3_bind_blob(&stmt, index, value.data(), static_cast<int>(value.size()), SQLITE_STATIC) != SQLITE_OK)
                throw std::runtime_error("Failed to bind blob to parameter " + std::to_string(index)
                                         + ": " + std::string(sqlite3_errmsg(&db)));
        }

        template <typename T>
        void bindParameter(sqlite3& db, sqlite3_stmt& stmt, const char* name, const T& value)
        {
            const int index = sqlite3_bind_parameter_index(&stmt, name);
            if (index == 0)
                throw std::logic_error("Parameter \"" + std::string(name) + "\" is not found");
            bindParameter(db, stmt, index, value);
        }

        std::vector<std::byte> compress(const std::vector<std::byte>& data)
        {
            const std::size_t originalSize = data.size();
            std::vector<std::byte> result(static_cast<std::size_t>(LZ4_compressBound(static_cast<int>(originalSize)) + sizeof(originalSize)));
            const int size = LZ4_compress_default(
                reinterpret_cast<const char*>(data.data()),
                reinterpret_cast<char*>(result.data()) + sizeof(originalSize),
                static_cast<int>(data.size()),
                static_cast<int>(result.size() - sizeof(originalSize))
            );
            if (size == 0)
                throw std::runtime_error("Failed to compress");
            std::memcpy(result.data(), &originalSize, sizeof(originalSize));
            result.resize(static_cast<std::size_t>(size) + sizeof(originalSize));
            return result;
        }

        std::vector<std::byte> decompress(const std::vector<std::byte>& data)
        {
            std::size_t originalSize;
            std::memcpy(&originalSize, data.data(), sizeof(originalSize));
            std::vector<std::byte> result(originalSize);
            const int size = LZ4_decompress_safe(
                reinterpret_cast<const char*>(data.data()) + sizeof(originalSize),
                reinterpret_cast<char*>(result.data()),
                static_cast<int>(data.size() - sizeof(originalSize)),
                static_cast<int>(result.size())
            );
            if (size < 0)
                throw std::runtime_error("Failed to decompress");
            if (originalSize != static_cast<std::size_t>(size))
                throw std::runtime_error("Size of decompressed data (" + std::to_string(size)
                                        + ") doesn't match stored (" + std::to_string(originalSize) + ")");
            return result;
        }
    }

    void CloseSqlite::operator()(sqlite3* handle) const noexcept
    {
        sqlite3_close(handle);
    }

    void CloseSqliteStmt::operator()(sqlite3_stmt* handle) const noexcept
    {
         sqlite3_finalize(handle);
    }

    Sqlite3StmtPtr makeStatement(sqlite3& db, std::string_view query)
    {
        sqlite3_stmt* stmt = nullptr;
        if (const int ec = sqlite3_prepare_v2(&db, query.data(), static_cast<int>(query.size()), &stmt, nullptr); ec != SQLITE_OK)
            throw std::runtime_error("Failed to prepare statement for query \"" + std::string(query) + "\": "
                                     + std::string(sqlite3_errmsg(&db)));
        return Sqlite3StmtPtr(stmt);
    }

    void Rollback::operator()(sqlite3* db) const
    {
        if (db == nullptr)
            return;
        if (const int ec = sqlite3_exec(db, "ROLLBACK", nullptr, nullptr, nullptr); ec != SQLITE_OK)
            Log(Debug::Warning) << "Failed to rollback SQLite3 transaction: " << std::string(sqlite3_errmsg(db));
    }

    Transaction::Transaction(sqlite3& db)
        : mDb(&db)
    {
        if (const int ec = sqlite3_exec(mDb.get(), "BEGIN", nullptr, nullptr, nullptr); ec != SQLITE_OK)
            throw std::runtime_error("Failed to start transaction: " + std::string(sqlite3_errmsg(mDb.get())));
    }

    void Transaction::commit()
    {
        if (const int ec = sqlite3_exec(mDb.get(), "COMMIT", nullptr, nullptr, nullptr); ec != SQLITE_OK)
            throw std::runtime_error("Failed to commit transaction: " + std::string(sqlite3_errmsg(mDb.get())));
        (void) mDb.release();
    }

    NavMeshDb::NavMeshDb(std::string_view path)
        : mDb(makeDb(path))
        , mGetMaxTileId(*mDb, DbQueries::GetMaxTileId {})
        , mFindTile(*mDb, DbQueries::FindTile {})
        , mGetTileData(*mDb, DbQueries::GetTileData {})
        , mInsertTile(*mDb, DbQueries::InsertTile {})
        , mUpdateTile(*mDb, DbQueries::UpdateTile {})
    {
    }

    Transaction NavMeshDb::startTransaction()
    {
        return Transaction(*mDb);
    }

    TileId NavMeshDb::getMaxTileId()
    {
        TileId tileId {0};
        request(*mDb, mGetMaxTileId, &tileId, 1);
        return tileId;
    }

    std::optional<Tile> NavMeshDb::findTile(const std::string& worldspace,
        const TilePosition& tilePosition, const std::vector<std::byte>& input)
    {
        Tile result;
        auto row = std::tie(result.mTileId, result.mVersion);
        const std::vector<std::byte> compressedInput = compress(input);
        if (&row == request(*mDb, mFindTile, &row, 1, worldspace, tilePosition, compressedInput))
            return {};
        return result;
    }

    std::optional<TileData> NavMeshDb::getTileData(const std::string& worldspace,
        const TilePosition& tilePosition, const std::vector<std::byte>& input)
    {
        TileData result;
        auto row = std::tie(result.mTileId, result.mVersion, result.mData);
        const std::vector<std::byte> compressedInput = compress(input);
        if (&row == request(*mDb, mGetTileData, &row, 1, worldspace, tilePosition, compressedInput))
            return {};
        result.mData = decompress(result.mData);
        return result;
    }

    int NavMeshDb::insertTile(TileId tileId, const std::string& worldspace, const TilePosition& tilePosition,
        TileVersion version, const std::vector<std::byte>& input, const std::vector<std::byte>& data)
    {
        const std::vector<std::byte> compressedInput = compress(input);
        const std::vector<std::byte> compressedData = compress(data);
        return execute(*mDb, mInsertTile, tileId, worldspace, tilePosition, version, compressedInput, compressedData);
    }

    int NavMeshDb::updateTile(TileId tileId, TileVersion version, const std::vector<std::byte>& data)
    {
        const std::vector<std::byte> compressedData = compress(data);
        return execute(*mDb, mUpdateTile, tileId, version, compressedData);
    }

    namespace DbQueries
    {
        std::string_view GetMaxTileId::text() noexcept
        {
            return getMaxTileIdQuery;
        }

        std::string_view FindTile::text() noexcept
        {
            return findTileQuery;
        }

        void FindTile::bind(sqlite3& db, sqlite3_stmt& statement, const std::string& worldspace,
            const TilePosition& tilePosition, const std::vector<std::byte>& input)
        {
            bindParameter(db, statement, ":worldspace", worldspace);
            bindParameter(db, statement, ":tile_position_x", tilePosition.x());
            bindParameter(db, statement, ":tile_position_y", tilePosition.y());
            bindParameter(db, statement, ":input", input);
        }

        std::string_view GetTileData::text() noexcept
        {
            return getTileDataQuery;
        }

        void GetTileData::bind(sqlite3& db, sqlite3_stmt& statement, const std::string& worldspace,
            const TilePosition& tilePosition, const std::vector<std::byte>& input)
        {
            bindParameter(db, statement, ":worldspace", worldspace);
            bindParameter(db, statement, ":tile_position_x", tilePosition.x());
            bindParameter(db, statement, ":tile_position_y", tilePosition.y());
            bindParameter(db, statement, ":input", input);
        }

        std::string_view InsertTile::text() noexcept
        {
            return insertTileQuery;
        }

        void InsertTile::bind(sqlite3& db, sqlite3_stmt& statement, TileId tileId, const std::string& worldspace,
            const TilePosition& tilePosition, TileVersion version, const std::vector<std::byte>& input,
            const std::vector<std::byte>& data)
        {
            bindParameter(db, statement, ":tile_id", tileId);
            bindParameter(db, statement, ":worldspace", worldspace);
            bindParameter(db, statement, ":tile_position_x", tilePosition.x());
            bindParameter(db, statement, ":tile_position_y", tilePosition.y());
            bindParameter(db, statement, ":version", version);
            bindParameter(db, statement, ":input", input);
            bindParameter(db, statement, ":data", data);
        }

        std::string_view UpdateTile::text() noexcept
        {
            return updateTileQuery;
        }

        void UpdateTile::bind(sqlite3& db, sqlite3_stmt& statement, TileId tileId, TileVersion version,
            const std::vector<std::byte>& data)
        {
            bindParameter(db, statement, ":tile_id", tileId);
            bindParameter(db, statement, ":version", version);
            bindParameter(db, statement, ":data", data);
        }
    }
}
