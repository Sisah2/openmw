#include "settings.hpp"

#include <components/detournavigator/asyncnavmeshupdater.hpp>
#include <components/detournavigator/makenavmesh.hpp>
#include <components/detournavigator/serialization.hpp>
#include <components/loadinglistener/loadinglistener.hpp>

#include <DetourNavMesh.h>

#include <gtest/gtest.h>

#include <map>

namespace
{
    using namespace testing;
    using namespace DetourNavigator;
    using namespace DetourNavigator::Tests;

    void addHeightFieldPlane(TileCachedRecastMeshManager& recastMeshManager)
    {
        const osg::Vec2i cellPosition(0, 0);
        const int cellSize = 8192;
        const osg::Vec3f shift(0, 0, 0);
        recastMeshManager.addHeightfield(cellPosition, cellSize, shift, HeightfieldPlane {0});
    }

    struct DetourNavigatorAsyncNavMeshUpdaterTest : Test
    {
        Settings mSettings = makeSettings();
        TileCachedRecastMeshManager mRecastMeshManager {mSettings};
        OffMeshConnectionsManager mOffMeshConnectionsManager {mSettings};
        const osg::Vec3f mAgentHalfExtents {29, 29, 66};
        const TilePosition mPlayerTile {0, 0};
        const std::string mWorldspace = "sys::default";
        Loading::Listener mListener;
    };

    TEST_F(DetourNavigatorAsyncNavMeshUpdaterTest, for_all_jobs_done_when_empty_wait_should_terminate)
    {
        AsyncNavMeshUpdater updater {mSettings, mRecastMeshManager, mOffMeshConnectionsManager, nullptr};
        updater.wait(mListener, WaitConditionType::allJobsDone);
    }

    TEST_F(DetourNavigatorAsyncNavMeshUpdaterTest, for_required_tiles_present_when_empty_wait_should_terminate)
    {
        AsyncNavMeshUpdater updater(mSettings, mRecastMeshManager, mOffMeshConnectionsManager, nullptr);
        updater.wait(mListener, WaitConditionType::requiredTilesPresent);
    }

    TEST_F(DetourNavigatorAsyncNavMeshUpdaterTest, post_should_generate_navmesh_tile)
    {
        mRecastMeshManager.setWorldspace(mWorldspace);
        addHeightFieldPlane(mRecastMeshManager);
        AsyncNavMeshUpdater updater(mSettings, mRecastMeshManager, mOffMeshConnectionsManager, nullptr);
        const auto navMeshCacheItem = std::make_shared<GuardedNavMeshCacheItem>(makeEmptyNavMesh(mSettings), 1);
        const std::map<TilePosition, ChangeType> changedTiles {{TilePosition {0, 0}, ChangeType::add}};
        updater.post(mAgentHalfExtents, navMeshCacheItem, mPlayerTile, mWorldspace, changedTiles);
        updater.wait(mListener, WaitConditionType::allJobsDone);
        EXPECT_NE(navMeshCacheItem->lockConst()->getImpl().getTileRefAt(0, 0, 0), 0);
    }

    TEST_F(DetourNavigatorAsyncNavMeshUpdaterTest, repeated_post_should_lead_to_cache_hit)
    {
        mRecastMeshManager.setWorldspace(mWorldspace);
        addHeightFieldPlane(mRecastMeshManager);
        AsyncNavMeshUpdater updater(mSettings, mRecastMeshManager, mOffMeshConnectionsManager, nullptr);
        const auto navMeshCacheItem = std::make_shared<GuardedNavMeshCacheItem>(makeEmptyNavMesh(mSettings), 1);
        const std::map<TilePosition, ChangeType> changedTiles {{TilePosition {0, 0}, ChangeType::add}};
        updater.post(mAgentHalfExtents, navMeshCacheItem, mPlayerTile, mWorldspace, changedTiles);
        updater.wait(mListener, WaitConditionType::allJobsDone);
        {
            const auto stats = updater.getStats();
            ASSERT_EQ(stats.mCache.mGetCount, 1);
            ASSERT_EQ(stats.mCache.mHitCount, 0);
        }
        updater.post(mAgentHalfExtents, navMeshCacheItem, mPlayerTile, mWorldspace, changedTiles);
        updater.wait(mListener, WaitConditionType::allJobsDone);
        {
            const auto stats = updater.getStats();
            EXPECT_EQ(stats.mCache.mGetCount, 2);
            EXPECT_EQ(stats.mCache.mHitCount, 1);
        }
    }

    TEST_F(DetourNavigatorAsyncNavMeshUpdaterTest, post_for_update_change_type_should_not_update_cache)
    {
        mRecastMeshManager.setWorldspace(mWorldspace);
        addHeightFieldPlane(mRecastMeshManager);
        AsyncNavMeshUpdater updater(mSettings, mRecastMeshManager, mOffMeshConnectionsManager, nullptr);
        const auto navMeshCacheItem = std::make_shared<GuardedNavMeshCacheItem>(makeEmptyNavMesh(mSettings), 1);
        const std::map<TilePosition, ChangeType> changedTiles {{TilePosition {0, 0}, ChangeType::update}};
        updater.post(mAgentHalfExtents, navMeshCacheItem, mPlayerTile, mWorldspace, changedTiles);
        updater.wait(mListener, WaitConditionType::allJobsDone);
        {
            const auto stats = updater.getStats();
            ASSERT_EQ(stats.mCache.mGetCount, 1);
            ASSERT_EQ(stats.mCache.mHitCount, 0);
        }
        updater.post(mAgentHalfExtents, navMeshCacheItem, mPlayerTile, mWorldspace, changedTiles);
        updater.wait(mListener, WaitConditionType::allJobsDone);
        {
            const auto stats = updater.getStats();
            EXPECT_EQ(stats.mCache.mGetCount, 2);
            EXPECT_EQ(stats.mCache.mHitCount, 0);
        }
    }

    TEST_F(DetourNavigatorAsyncNavMeshUpdaterTest, post_should_write_generated_tile_to_db)
    {
        mRecastMeshManager.setWorldspace(mWorldspace);
        addHeightFieldPlane(mRecastMeshManager);
        auto db = std::make_unique<NavMeshDb>(":memory:");
        NavMeshDb* const dbPtr = db.get();
        AsyncNavMeshUpdater updater(mSettings, mRecastMeshManager, mOffMeshConnectionsManager, std::move(db));
        const auto navMeshCacheItem = std::make_shared<GuardedNavMeshCacheItem>(makeEmptyNavMesh(mSettings), 1);
        const TilePosition tilePosition {0, 0};
        const std::map<TilePosition, ChangeType> changedTiles {{tilePosition, ChangeType::add}};
        updater.post(mAgentHalfExtents, navMeshCacheItem, mPlayerTile, mWorldspace, changedTiles);
        updater.wait(mListener, WaitConditionType::allJobsDone);
        const Bounds bounds {osg::Vec3f(-3.2, 0, -3.2), osg::Vec3f(16, 0, 16)};
        const rcConfig config = makeRecastConfig(tilePosition, bounds, mAgentHalfExtents, mSettings);
        const auto recastMesh = mRecastMeshManager.getMesh(mWorldspace, tilePosition);
        ASSERT_NE(recastMesh, nullptr);
        const auto tile = dbPtr->findTile(mWorldspace, tilePosition, serialize(mSettings.mRecastScaleFactor, config, *recastMesh));
        ASSERT_TRUE(tile.has_value());
        EXPECT_EQ(tile->mTileId, 1);
        EXPECT_EQ(tile->mVersion, mSettings.mNavMeshVersion);
    }

    TEST_F(DetourNavigatorAsyncNavMeshUpdaterTest, post_when_writing_to_db_disabled_should_not_write)
    {
        mRecastMeshManager.setWorldspace(mWorldspace);
        addHeightFieldPlane(mRecastMeshManager);
        auto db = std::make_unique<NavMeshDb>(":memory:");
        NavMeshDb* const dbPtr = db.get();
        mSettings.mWriteToNavMeshDb = false;
        AsyncNavMeshUpdater updater(mSettings, mRecastMeshManager, mOffMeshConnectionsManager, std::move(db));
        const auto navMeshCacheItem = std::make_shared<GuardedNavMeshCacheItem>(makeEmptyNavMesh(mSettings), 1);
        const TilePosition tilePosition {0, 0};
        const std::map<TilePosition, ChangeType> changedTiles {{tilePosition, ChangeType::add}};
        updater.post(mAgentHalfExtents, navMeshCacheItem, mPlayerTile, mWorldspace, changedTiles);
        updater.wait(mListener, WaitConditionType::allJobsDone);
        const Bounds bounds {osg::Vec3f(-3.2, 0, -3.2), osg::Vec3f(16, 0, 16)};
        const rcConfig config = makeRecastConfig(tilePosition, bounds, mAgentHalfExtents, mSettings);
        const auto recastMesh = mRecastMeshManager.getMesh(mWorldspace, tilePosition);
        ASSERT_NE(recastMesh, nullptr);
        const auto tile = dbPtr->findTile(mWorldspace, tilePosition, serialize(mSettings.mRecastScaleFactor, config, *recastMesh));
        ASSERT_FALSE(tile.has_value());
    }

    TEST_F(DetourNavigatorAsyncNavMeshUpdaterTest, post_should_read_from_db_on_cache_miss)
    {
        mRecastMeshManager.setWorldspace(mWorldspace);
        addHeightFieldPlane(mRecastMeshManager);
        mSettings.mMaxNavMeshTilesCacheSize = 0;
        AsyncNavMeshUpdater updater(mSettings, mRecastMeshManager, mOffMeshConnectionsManager, std::make_unique<NavMeshDb>(":memory:"));
        const auto navMeshCacheItem = std::make_shared<GuardedNavMeshCacheItem>(makeEmptyNavMesh(mSettings), 1);
        const std::map<TilePosition, ChangeType> changedTiles {{TilePosition {0, 0}, ChangeType::add}};
        updater.post(mAgentHalfExtents, navMeshCacheItem, mPlayerTile, mWorldspace, changedTiles);
        updater.wait(mListener, WaitConditionType::allJobsDone);
        {
            const auto stats = updater.getStats();
            ASSERT_EQ(stats.mCache.mGetCount, 1);
            ASSERT_EQ(stats.mCache.mHitCount, 0);
            ASSERT_TRUE(stats.mDb.has_value());
            ASSERT_EQ(stats.mDb->mGetTileCount, 1);
            ASSERT_EQ(stats.mDbGetTileHits, 0);
        }
        updater.post(mAgentHalfExtents, navMeshCacheItem, mPlayerTile, mWorldspace, changedTiles);
        updater.wait(mListener, WaitConditionType::allJobsDone);
        {
            const auto stats = updater.getStats();
            EXPECT_EQ(stats.mCache.mGetCount, 2);
            EXPECT_EQ(stats.mCache.mHitCount, 0);
            ASSERT_TRUE(stats.mDb.has_value());
            EXPECT_EQ(stats.mDb->mGetTileCount, 2);
            EXPECT_EQ(stats.mDbGetTileHits, 1);
        }
    }

    TEST_F(DetourNavigatorAsyncNavMeshUpdaterTest, on_changing_player_tile_post_should_remove_tiles_out_of_range)
    {
        mRecastMeshManager.setWorldspace(mWorldspace);
        addHeightFieldPlane(mRecastMeshManager);
        AsyncNavMeshUpdater updater(mSettings, mRecastMeshManager, mOffMeshConnectionsManager, nullptr);
        const auto navMeshCacheItem = std::make_shared<GuardedNavMeshCacheItem>(makeEmptyNavMesh(mSettings), 1);
        const std::map<TilePosition, ChangeType> changedTilesAdd {{TilePosition {0, 0}, ChangeType::add}};
        updater.post(mAgentHalfExtents, navMeshCacheItem, mPlayerTile, mWorldspace, changedTilesAdd);
        updater.wait(mListener, WaitConditionType::allJobsDone);
        ASSERT_NE(navMeshCacheItem->lockConst()->getImpl().getTileRefAt(0, 0, 0), 0);
        const std::map<TilePosition, ChangeType> changedTilesRemove {{TilePosition {0, 0}, ChangeType::remove}};
        const TilePosition playerTile(100, 100);
        updater.post(mAgentHalfExtents, navMeshCacheItem, playerTile, mWorldspace, changedTilesRemove);
        updater.wait(mListener, WaitConditionType::allJobsDone);
        EXPECT_EQ(navMeshCacheItem->lockConst()->getImpl().getTileRefAt(0, 0, 0), 0);
    }
}
