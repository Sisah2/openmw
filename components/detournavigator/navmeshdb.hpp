#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_NAVMESHDB_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_NAVMESHDB_H

#include "tileposition.hpp"

#include <SQLiteCpp/Database.h>
#include <SQLiteCpp/Statement.h>
#include <SQLiteCpp/Transaction.h>

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
            static const char* text() noexcept;
            static void bind(SQLite::Statement&) {}
        };

        struct FindTile
        {
            static const char* text() noexcept;
            static void bind(SQLite::Statement& statement, const std::string& worldspace, const TilePosition& tilePosition,
                const std::vector<std::byte>& input);
        };

        struct GetTileData
        {
            static const char* text() noexcept;
            static void bind(SQLite::Statement& statement, const std::string& worldspace, const TilePosition& tilePosition,
                const std::vector<std::byte>& input);
        };

        struct InsertTile
        {
            static const char* text() noexcept;
            static void bind(SQLite::Statement& statement, TileId tileId, const std::string& worldspace, const TilePosition& tilePosition,
                TileVersion version, const std::vector<std::byte>& input, const std::vector<std::byte>& data);
        };

        struct UpdateTile
        {
            static const char* text() noexcept;
            static void bind(SQLite::Statement& statement, TileId tileId, TileVersion version, const std::vector<std::byte>& data);
        };
    }

    template <class Query>
    struct SqliteStatement
    {
        bool mNeedReset = false;
        std::unique_ptr<SQLite::Statement> mStatement;
        Query mQuery;

        explicit SqliteStatement(SQLite::Database& db, Query query = Query {})
            : mStatement(std::make_unique<SQLite::Statement>(db, query.text())),
              mQuery(std::move(query)) {}
    };

    class NavMeshDb
    {
    public:
        explicit NavMeshDb(std::string_view path);

        std::unique_ptr<SQLite::Transaction> startTransaction();

        TileId getMaxTileId();

        std::optional<Tile> findTile(const std::string& worldspace,
            const TilePosition& tilePosition, const std::vector<std::byte>& input);

        std::optional<TileData> getTileData(const std::string& worldspace,
            const TilePosition& tilePosition, const std::vector<std::byte>& input);

        int insertTile(TileId tileId, const std::string& worldspace, const TilePosition& tilePosition,
            TileVersion version, const std::vector<std::byte>& input, const std::vector<std::byte>& data);

        int updateTile(TileId tileId, TileVersion version, const std::vector<std::byte>& data);

    private:
        SQLite::Database mDb;
        SqliteStatement<DbQueries::GetMaxTileId> mGetMaxTileId;
        SqliteStatement<DbQueries::FindTile> mFindTile;
        SqliteStatement<DbQueries::GetTileData> mGetTileData;
        SqliteStatement<DbQueries::InsertTile> mInsertTile;
        SqliteStatement<DbQueries::UpdateTile> mUpdateTile;
    };
}

#endif
