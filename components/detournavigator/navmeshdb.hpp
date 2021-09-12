#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_NAVMESHDB_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_NAVMESHDB_H

#include "tileposition.hpp"

#include <boost/serialization/strong_typedef.hpp>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>
#include <memory>

struct sqlite3;
struct sqlite3_stmt;

namespace DetourNavigator
{
    BOOST_STRONG_TYPEDEF(std::int64_t, TileId)
    BOOST_STRONG_TYPEDEF(std::int64_t, TileRevision)
    BOOST_STRONG_TYPEDEF(std::int64_t, TileVersion)

    struct Tile
    {
        TileId mTileId;
        TileVersion mVersion;
    };

    struct TileData
    {
        TileId mTileId;
        TileVersion mVersion;
        std::vector<std::byte> mData;
    };

    namespace DbQueries
    {
        struct GetMaxTileId
        {
            static std::string_view text() noexcept;
            static void bind(sqlite3&, sqlite3_stmt&) {}
        };

        struct FindTile
        {
            static std::string_view text() noexcept;
            static void bind(sqlite3& db, sqlite3_stmt& statement, const std::string& worldspace,
                const TilePosition& tilePosition, const std::vector<std::byte>& input);
        };

        struct GetTileData
        {
            static std::string_view text() noexcept;
            static void bind(sqlite3& db, sqlite3_stmt& statement, const std::string& worldspace,
                const TilePosition& tilePosition, const std::vector<std::byte>& input);
        };

        struct InsertTile
        {
            static std::string_view text() noexcept;
            static void bind(sqlite3& db, sqlite3_stmt& statement, TileId tileId, const std::string& worldspace,
                const TilePosition& tilePosition, TileVersion version, const std::vector<std::byte>& input,
                const std::vector<std::byte>& data);
        };

        struct UpdateTile
        {
            static std::string_view text() noexcept;
            static void bind(sqlite3& db, sqlite3_stmt& statement, TileId tileId, TileVersion version,
                const std::vector<std::byte>& data);
        };
    }

    struct CloseSqlite
    {
        void operator()(sqlite3* handle) const noexcept;
    };

    using Sqlite3Ptr = std::unique_ptr<sqlite3, CloseSqlite>;

    struct CloseSqliteStmt
    {
        void operator()(sqlite3_stmt* handle) const noexcept;
    };

    using Sqlite3StmtPtr = std::unique_ptr<sqlite3_stmt, CloseSqliteStmt>;

    Sqlite3StmtPtr makeStatement(sqlite3& db, std::string_view query);

    template <class Query>
    struct SqliteStatement
    {
        bool mNeedReset = false;
        Sqlite3StmtPtr mStatement;
        Query mQuery;

        explicit SqliteStatement(sqlite3& db, Query query = Query {})
            : mStatement(makeStatement(db, query.text())),
              mQuery(std::move(query)) {}
    };

    struct Rollback
    {
        void operator()(sqlite3* handle) const;
    };

    class Transaction
    {
    public:
        Transaction(sqlite3& db);

        void commit();

    private:
        std::unique_ptr<sqlite3, Rollback> mDb;
    };

    class NavMeshDb
    {
    public:
        explicit NavMeshDb(std::string_view path);

        Transaction startTransaction();

        TileId getMaxTileId();

        std::optional<Tile> findTile(const std::string& worldspace,
            const TilePosition& tilePosition, const std::vector<std::byte>& input);

        std::optional<TileData> getTileData(const std::string& worldspace,
            const TilePosition& tilePosition, const std::vector<std::byte>& input);

        int insertTile(TileId tileId, const std::string& worldspace, const TilePosition& tilePosition,
            TileVersion version, const std::vector<std::byte>& input, const std::vector<std::byte>& data);

        int updateTile(TileId tileId, TileVersion version, const std::vector<std::byte>& data);

    private:
        Sqlite3Ptr mDb;
        SqliteStatement<DbQueries::GetMaxTileId> mGetMaxTileId;
        SqliteStatement<DbQueries::FindTile> mFindTile;
        SqliteStatement<DbQueries::GetTileData> mGetTileData;
        SqliteStatement<DbQueries::InsertTile> mInsertTile;
        SqliteStatement<DbQueries::UpdateTile> mUpdateTile;
    };
}

#endif
