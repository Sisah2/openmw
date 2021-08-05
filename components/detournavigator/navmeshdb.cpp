#include "navmeshdb.hpp"

#include <lz4.h>

#include <DetourAlloc.h>

#include <SQLiteCpp/Database.h>
#include <SQLiteCpp/Statement.h>
#include <SQLiteCpp/Transaction.h>

#include <cstddef>
#include <string>
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

        constexpr const char getMaxTileIdQuery[] = R"(
            SELECT max(tile_id) FROM tiles
        )";

        constexpr const char findTileQuery[] = R"(
            SELECT tile_id, version
            FROM tiles
            WHERE worldspace = :worldspace
            AND tile_position_x = :tile_position_x
            AND tile_position_y = :tile_position_y
            AND input = :input
        )";

        constexpr const char getTileDataQuery[] = R"(
            SELECT tile_id, version, data
            FROM tiles
            WHERE worldspace = :worldspace
            AND tile_position_x = :tile_position_x
            AND tile_position_y = :tile_position_y
            AND input = :input
        )";

        constexpr const char insertTileQuery[] = R"(
            INSERT INTO tiles ( tile_id,  worldspace,  version,  tile_position_x,  tile_position_y,  input,  data)
                    VALUES (:tile_id, :worldspace, :version, :tile_position_x, :tile_position_y, :input, :data)
        )";

        constexpr const char updateTileQuery[] = R"(
            UPDATE tiles
            SET version = :version,
                data = :data,
                revision = revision + 1
            WHERE tile_id = :tile_id
        )";

        template <class T>
        void copyColumn(const SQLite::Column& column, T& value)
        {
            value = static_cast<T>(column);
        }

        void copyColumn(const SQLite::Column& column, std::vector<std::byte>& value)
        {
            const int size = column.getBytes();
            if (size > 0)
            {
                const void* const blob = column.getBlob();
                value.reserve(static_cast<std::size_t>(size));
                value.assign(static_cast<const std::byte*>(blob), static_cast<const std::byte*>(blob) + size);
            }
            else
            {
                value.clear();
            }
        }

        template <int index, class T>
        void getColumnsImpl(SQLite::Statement& statement, T& row)
        {
            if constexpr (0 < index && index <= std::tuple_size_v<T>)
            {
                const SQLite::Column& column = statement.getColumn(index - 1);
                if (column.isNull())
                    std::get<index - 1>(row) = std::decay_t<decltype(std::get<index - 1>(row))> {};
                else
                    copyColumn(column, std::get<index - 1>(row));
                getColumnsImpl<index - 1>(statement, row);
            }
        }

        template <class T>
        void getColumns(SQLite::Statement& statement, T& row)
        {
            getColumnsImpl<std::tuple_size_v<T>>(statement, row);
        }

        template <class T>
        void getRow(SQLite::Statement& statement, T& row)
        {
            auto tuple = std::tie(row);
            getColumns(statement, tuple);
        }

        template <class ... Args>
        void getRow(SQLite::Statement& statement, std::tuple<Args ...>& row)
        {
            getColumns(statement, row);
        }

        template <class T, class ... Args>
        void prepare(SQLite::Database& db, SqliteStatement<T>& statement, Args&& ... args)
        {
            if (statement.mNeedReset)
            {
                if (statement.mStatement->tryReset() == SQLite::OK)
                    statement.mStatement->clearBindings();
                else
                    statement.mStatement = std::make_unique<SQLite::Statement>(db, statement.mQuery.text());
                statement.mNeedReset = false;
            }
            statement.mQuery.bind(*statement.mStatement, std::forward<Args>(args) ...);
        }

        template <class T, class I, class ... Args>
        I request(SQLite::Database& db, SqliteStatement<T>& statement, I out, std::size_t max, Args&& ... args)
        {
            try
            {
                statement.mNeedReset = true;
                prepare(db, statement, std::forward<Args>(args) ...);
                for (std::size_t i = 0; statement.mStatement->executeStep() && i < max; ++i)
                    getRow(*statement.mStatement, *out++);
                return out;
            }
            catch (const SQLite::Exception& e)
            {
                throw std::runtime_error("Failed perform request \"" + std::string(statement.mQuery.text())
                                        + "\": " + std::string(e.what()));
            }
        }

        template <class T, class ... Args>
        int execute(SQLite::Database& db, SqliteStatement<T>& statement, Args&& ... args)
        {
            try
            {
                statement.mNeedReset = true;
                prepare(db, statement, std::forward<Args>(args) ...);
                const int result = statement.mStatement->exec();
                return result;
            }
            catch (const SQLite::Exception& e)
            {
                throw std::runtime_error("Failed to execute statement \"" + std::string(statement.mQuery.text())
                                        + "\": " + std::string(e.what()));
            }
        }

        SQLite::Database makeDb(std::string_view path)
        {
            SQLite::Database db(path, SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE);
            db.exec(schema);
            return db;
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

    NavMeshDb::NavMeshDb(std::string_view path)
        : mDb(makeDb(path))
        , mGetMaxTileId(mDb, DbQueries::GetMaxTileId {})
        , mFindTile(mDb, DbQueries::FindTile {})
        , mGetTileData(mDb, DbQueries::GetTileData {})
        , mInsertTile(mDb, DbQueries::InsertTile {})
        , mUpdateTile(mDb, DbQueries::UpdateTile {})
    {
    }

    std::unique_ptr<SQLite::Transaction> NavMeshDb::startTransaction()
    {
        return std::make_unique<SQLite::Transaction>(mDb);
    }

    TileId NavMeshDb::getMaxTileId()
    {
        TileId tileId {0};
        request(mDb, mGetMaxTileId, &tileId, 1);
        return tileId;
    }

    std::optional<Tile> NavMeshDb::findTile(const std::string& worldspace,
        const TilePosition& tilePosition, const std::vector<std::byte>& input)
    {
        Tile result;
        auto row = std::tie(result.mTileId, result.mVersion);
        const std::vector<std::byte> compressedInput = compress(input);
        if (&row == request(mDb, mFindTile, &row, 1, worldspace, tilePosition, compressedInput))
            return {};
        return result;
    }

    std::optional<TileData> NavMeshDb::getTileData(const std::string& worldspace,
        const TilePosition& tilePosition, const std::vector<std::byte>& input)
    {
        TileData result;
        auto row = std::tie(result.mTileId, result.mVersion, result.mData);
        const std::vector<std::byte> compressedInput = compress(input);
        if (&row == request(mDb, mGetTileData, &row, 1, worldspace, tilePosition, compressedInput))
            return {};
        result.mData = decompress(result.mData);
        return result;
    }

    int NavMeshDb::insertTile(TileId tileId, const std::string& worldspace, const TilePosition& tilePosition,
        TileVersion version, const std::vector<std::byte>& input, const std::vector<std::byte>& data)
    {
        const std::vector<std::byte> compressedInput = compress(input);
        const std::vector<std::byte> compressedData = compress(data);
        return execute(mDb, mInsertTile, tileId, worldspace, tilePosition, version, compressedInput, compressedData);
    }

    int NavMeshDb::updateTile(TileId tileId, TileVersion version, const std::vector<std::byte>& data)
    {
        const std::vector<std::byte> compressedData = compress(data);
        return execute(mDb, mUpdateTile, tileId, version, compressedData);
    }

    namespace DbQueries
    {
        const char* GetMaxTileId::text() noexcept
        {
            return getMaxTileIdQuery;
        }

        const char* FindTile::text() noexcept
        {
            return findTileQuery;
        }

        void FindTile::bind(SQLite::Statement& statement, const std::string& worldspace, const TilePosition& tilePosition,
            const std::vector<std::byte>& input)
        {
            statement.bind(":worldspace", worldspace);
            statement.bind(":tile_position_x", tilePosition.x());
            statement.bind(":tile_position_y", tilePosition.y());
            statement.bindNoCopy(":input", input.data(), static_cast<int>(input.size()));
        }

        const char* GetTileData::text() noexcept
        {
            return getTileDataQuery;
        }

        void GetTileData::bind(SQLite::Statement& statement, const std::string& worldspace, const TilePosition& tilePosition,
            const std::vector<std::byte>& input)
        {
            statement.bind(":worldspace", worldspace);
            statement.bind(":tile_position_x", tilePosition.x());
            statement.bind(":tile_position_y", tilePosition.y());
            statement.bindNoCopy(":input", input.data(), static_cast<int>(input.size()));
        }

        const char* InsertTile::text() noexcept
        {
            return insertTileQuery;
        }

        void InsertTile::bind(SQLite::Statement& statement, TileId tileId, const std::string& worldspace,
            const TilePosition& tilePosition, TileVersion version, const std::vector<std::byte>& input, const std::vector<std::byte>& data)
        {
            statement.bind(":tile_id", tileId);
            statement.bind(":worldspace", worldspace);
            statement.bind(":tile_position_x", tilePosition.x());
            statement.bind(":tile_position_y", tilePosition.y());
            statement.bind(":version", version);
            statement.bindNoCopy(":input", input.data(), static_cast<int>(input.size()));
            statement.bindNoCopy(":data", data.data(), static_cast<int>(data.size()));
        }

        const char* UpdateTile::text() noexcept
        {
            return updateTileQuery;
        }

        void UpdateTile::bind(SQLite::Statement& statement, TileId tileId, TileVersion version, const std::vector<std::byte>& data)
        {
            statement.bind(":tile_id", tileId);
            statement.bind(":version", version);
            statement.bindNoCopy(":data", data.data(), static_cast<int>(data.size()));
        }
    }
}
