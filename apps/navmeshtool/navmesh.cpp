#include "navmesh.hpp"

#include "worldspacedata.hpp"

#include <components/bullethelpers/aabb.hpp>
#include <components/debug/debuglog.hpp>
#include <components/detournavigator/tileposition.hpp>
#include <components/detournavigator/generatenavmeshtile.hpp>
#include <components/detournavigator/gettilespositions.hpp>
#include <components/detournavigator/offmeshconnection.hpp>
#include <components/detournavigator/offmeshconnectionsmanager.hpp>
#include <components/detournavigator/preparednavmeshdata.hpp>
#include <components/detournavigator/recastmesh.hpp>
#include <components/detournavigator/serialization.hpp>
#include <components/detournavigator/navmeshdb.hpp>
#include <components/detournavigator/tilecachedrecastmeshmanager.hpp>
#include <components/detournavigator/recastmeshprovider.hpp>
#include <components/esm/loadcell.hpp>
#include <components/misc/guarded.hpp>
#include <components/sceneutil/workqueue.hpp>

#include <DetourNavMesh.h>

#include <SQLiteCpp/Transaction.h>

#include <osg/Vec3f>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace NavMeshTool
{
    namespace
    {
        using DetourNavigator::GenerateNavMeshTile;
        using DetourNavigator::NavMeshTileInfo;
        using DetourNavigator::PreparedNavMeshData;
        using DetourNavigator::RecastMeshProvider;
        using DetourNavigator::Settings;
        using DetourNavigator::NavMeshDb;
        using DetourNavigator::TileId;
        using DetourNavigator::TilePosition;
        using DetourNavigator::TileVersion;

        void logGeneratedTiles(std::size_t provided, std::size_t expected)
        {
            Log(Debug::Info) << provided << "/" << expected << " ("
                << (static_cast<double>(provided) / static_cast<double>(expected) * 100)
                << "%) navmesh tiles are generated";
        }

        class ProgressReporter
        {
        public:
            void report(std::size_t provided, std::size_t expected)
            {
                expected = std::max(expected, provided);
                const bool shouldReport = [&]
                {
                    const auto nextReport = mNextReport.lock();
                    const auto now = std::chrono::steady_clock::now();
                    const auto left = *nextReport - now;
                    if (left.count() > 0 || provided == expected)
                        return false;
                    *nextReport += std::chrono::seconds(1) + std::chrono::duration_cast<std::chrono::seconds>(left);
                    return true;
                } ();
                if (shouldReport)
                    logGeneratedTiles(provided, expected);
            }

        private:
            Misc::ScopeGuarded<std::chrono::steady_clock::time_point> mNextReport {std::chrono::steady_clock::now()};
        };

        class NavMeshTileConsumer final : public DetourNavigator::NavMeshTileConsumer
        {
        public:
            std::atomic_size_t mExpected {0};

            explicit NavMeshTileConsumer(NavMeshDb db)
                : mDb(std::move(db))
                , mTransaction(mDb.startTransaction())
                , mNextTileId(mDb.getMaxTileId())
            {
                ++mNextTileId.t;
            }

            std::optional<NavMeshTileInfo> find(const std::string& worldspace, const TilePosition &tilePosition,
                const std::vector<std::byte> &input) override
            {
                std::optional<NavMeshTileInfo> result;
                std::lock_guard lock(mMutex);
                if (const auto tile = mDb.findTile(worldspace, tilePosition, input))
                {
                    NavMeshTileInfo info;
                    info.mTileId = tile->mTileId;
                    info.mVersion = tile->mVersion;
                    result.emplace(info);
                }
                return result;
            }

            void ignore() override
            {
                mReporter.report(++mProvided, mExpected);
                mHasTile.notify_one();
            }

            void insert(const std::string& worldspace, const TilePosition& tilePosition, std::int64_t version,
                const std::vector<std::byte>& input, PreparedNavMeshData& data) override
            {
                mReporter.report(++mProvided, mExpected);
                data.mUserId = static_cast<unsigned>(mNextTileId);
                std::lock_guard lock(mMutex);
                mDb.insertTile(mNextTileId, worldspace, tilePosition, TileVersion {version}, input, serialize(data));
                ++mNextTileId.t;
                mHasTile.notify_one();
            }

            void update(std::int64_t tileId, std::int64_t version, PreparedNavMeshData& data) override
            {
                mReporter.report(++mProvided, mExpected);
                data.mUserId = static_cast<unsigned>(tileId);
                std::lock_guard lock(mMutex);
                mDb.updateTile(TileId {tileId}, TileVersion {version}, serialize(data));
                mHasTile.notify_one();
            }

            void wait()
            {
                std::unique_lock lock(mMutex);
                mHasTile.wait(lock, [&] { return mExpected <= mProvided; });
                logGeneratedTiles(mProvided, mExpected);
            }

            void commit()
            {
                mTransaction->commit();
            }

        private:
            std::atomic_size_t mProvided {0};
            std::mutex mMutex;
            NavMeshDb mDb;
            std::unique_ptr<SQLite::Transaction> mTransaction;
            TileId mNextTileId;
            std::condition_variable mHasTile;
            ProgressReporter mReporter;
        };
    }

    void generateAllNavMeshTiles(const osg::Vec3f& agentHalfExtents, const Settings& settings,
        const std::size_t threadsNumber, WorldspaceData& data, NavMeshDb&& db)
    {
        Log(Debug::Info) << "Generating navmesh tiles by " << threadsNumber << " parallel workers...";

        SceneUtil::WorkQueue workQueue(threadsNumber);
        auto navMeshTileConsumer = std::make_shared<NavMeshTileConsumer>(std::move(db));
        std::size_t tiles = 0;

        for (const std::unique_ptr<WorldspaceNavMeshInput>& input : data.mNavMeshInputs)
        {
            DetourNavigator::getTilesPositions(
                Misc::Convert::toOsg(input->mAabb.m_min), Misc::Convert::toOsg(input->mAabb.m_max), settings,
                [&] (const TilePosition& tilePosition)
                {
                    workQueue.addWorkItem(new GenerateNavMeshTile(
                        input->mWorldspace,
                        tilePosition,
                        RecastMeshProvider(input->mTileCachedRecastMeshManager),
                        agentHalfExtents,
                        settings,
                        navMeshTileConsumer
                    ));

                    ++tiles;
                });

            navMeshTileConsumer->mExpected = tiles;
        }

        navMeshTileConsumer->wait();
        navMeshTileConsumer->commit();
    }
}
