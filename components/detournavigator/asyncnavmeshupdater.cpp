#include "asyncnavmeshupdater.hpp"
#include "debug.hpp"
#include "makenavmesh.hpp"
#include "settings.hpp"
#include "version.hpp"
#include "serialization.hpp"

#include <components/debug/debuglog.hpp>
#include <components/misc/thread.hpp>
#include <components/loadinglistener/loadinglistener.hpp>

#include <DetourNavMesh.h>

#include <osg/Stats>

#include <algorithm>
#include <numeric>
#include <set>
#include <type_traits>

namespace DetourNavigator
{
    namespace
    {
        int getManhattanDistance(const TilePosition& lhs, const TilePosition& rhs)
        {
            return std::abs(lhs.x() - rhs.x()) + std::abs(lhs.y() - rhs.y());
        }

        int getMinDistanceTo(const TilePosition& position, int maxDistance,
                             const std::set<std::tuple<osg::Vec3f, TilePosition>>& pushedTiles,
                             const std::set<std::tuple<osg::Vec3f, TilePosition>>& presentTiles)
        {
            int result = maxDistance;
            for (const auto& [halfExtents, tile] : pushedTiles)
                if (presentTiles.find(std::tie(halfExtents, tile)) == presentTiles.end())
                    result = std::min(result, getManhattanDistance(position, tile));
            return result;
        }

        auto getPriority(const Job& job) noexcept
        {
            return std::make_tuple(-static_cast<std::underlying_type_t<JobState>>(job.mState), job.mProcessTime,
                                   job.mChangeType, job.mTryNumber, job.mDistanceToPlayer, job.mDistanceToOrigin);
        }

        struct LessByJobPriority
        {
            bool operator()(JobIt lhs, JobIt rhs) const noexcept
            {
                return getPriority(*lhs) < getPriority(*rhs);
            }
        };

        void insertPrioritizedJob(JobIt job, std::deque<JobIt>& queue)
        {
            const auto it = std::upper_bound(queue.begin(), queue.end(), job, LessByJobPriority {});
            queue.insert(it, job);
        }

        auto getDbPriority(const Job& job) noexcept
        {
            return std::make_tuple(static_cast<std::underlying_type_t<JobState>>(job.mState),
                                   job.mChangeType, job.mDistanceToPlayer, job.mDistanceToOrigin);
        }

        struct LessByJobDbPriority
        {
            bool operator()(JobIt lhs, JobIt rhs) const noexcept
            {
                return getDbPriority(*lhs) < getDbPriority(*rhs);
            }
        };

        void insertPrioritizedDbJob(JobIt job, std::deque<JobIt>& queue)
        {
            const auto it = std::upper_bound(queue.begin(), queue.end(), job, LessByJobDbPriority {});
            queue.insert(it, job);
        }

        auto getAgentAndTile(const Job& job) noexcept
        {
            return std::make_tuple(job.mAgentHalfExtents, job.mChangedTile);
        }

        std::unique_ptr<DbWorker> makeDbWorker(AsyncNavMeshUpdater& updater, std::unique_ptr<NavMeshDb>&& db, const Settings& settings)
        {
            if (db == nullptr)
                return nullptr;
            return std::make_unique<DbWorker>(updater, std::move(db), TileVersion(settings.mNavMeshVersion));
        }

        void updateJobs(std::deque<JobIt>& jobs, TilePosition playerTile, int maxTiles)
        {
            for (JobIt job : jobs)
            {
                job->mDistanceToPlayer = getManhattanDistance(job->mChangedTile, playerTile);
                if (!shouldAddTile(job->mChangedTile, playerTile, maxTiles))
                    job->mChangeType = ChangeType::remove;
            }
        }

        class RemoveJob
        {
        public:
            RemoveJob(AsyncNavMeshUpdater& updater, JobIt job)
                : mUpdater(&updater), mJob(job) {}

            RemoveJob(const RemoveJob&) = delete;
            RemoveJob(RemoveJob&&) = delete;

            ~RemoveJob()
            {
                if (mUpdater != nullptr)
                    mUpdater->removeJob(mJob);
            }

            RemoveJob& operator=(const RemoveJob&) = delete;
            RemoveJob& operator=(RemoveJob&&) = delete;

            void cancel() { mUpdater = nullptr; };

        private:
            AsyncNavMeshUpdater* mUpdater;
            JobIt mJob;
        };
    }

    Job::Job(const osg::Vec3f& agentHalfExtents, std::weak_ptr<GuardedNavMeshCacheItem> navMeshCacheItem,
        std::string_view worldspace, const TilePosition& changedTile, ChangeType changeType, int distanceToPlayer,
        std::chrono::steady_clock::time_point processTime)
        : mAgentHalfExtents(agentHalfExtents)
        , mNavMeshCacheItem(std::move(navMeshCacheItem))
        , mWorldspace(worldspace)
        , mChangedTile(changedTile)
        , mProcessTime(processTime)
        , mChangeType(changeType)
        , mDistanceToPlayer(distanceToPlayer)
        , mDistanceToOrigin(getManhattanDistance(changedTile, TilePosition {0, 0}))
    {
    }

    AsyncNavMeshUpdater::AsyncNavMeshUpdater(const Settings& settings, TileCachedRecastMeshManager& recastMeshManager,
            OffMeshConnectionsManager& offMeshConnectionsManager, std::unique_ptr<NavMeshDb>&& db)
        : mSettings(settings)
        , mRecastMeshManager(recastMeshManager)
        , mOffMeshConnectionsManager(offMeshConnectionsManager)
        , mShouldStop()
        , mNavMeshTilesCache(settings.mMaxNavMeshTilesCacheSize)
        , mDbWorker(makeDbWorker(*this, std::move(db), mSettings))
    {
        for (std::size_t i = 0; i < mSettings.get().mAsyncNavMeshUpdaterThreads; ++i)
            mThreads.emplace_back([&] { process(); });
    }

    AsyncNavMeshUpdater::~AsyncNavMeshUpdater()
    {
        mShouldStop = true;
        if (mDbWorker != nullptr)
            mDbWorker->stop();
        std::unique_lock<std::mutex> lock(mMutex);
        mWaiting.clear();
        mHasJob.notify_all();
        lock.unlock();
        for (auto& thread : mThreads)
            thread.join();
    }

    void AsyncNavMeshUpdater::post(const osg::Vec3f& agentHalfExtents, const SharedNavMeshCacheItem& navMeshCacheItem,
        const TilePosition& playerTile, std::string_view worldspace,
        const std::map<TilePosition, ChangeType>& changedTiles)
    {
        bool playerTileChanged = false;
        {
            auto locked = mPlayerTile.lock();
            playerTileChanged = *locked != playerTile;
            *locked = playerTile;
        }

        if (!playerTileChanged && changedTiles.empty())
            return;

        const dtNavMeshParams params = *navMeshCacheItem->lockConst()->getImpl().getParams();
        const int maxTiles = std::min(mSettings.get().mMaxTilesNumber, params.maxTiles);

        std::unique_lock lock(mMutex);

        if (playerTileChanged)
            updateJobs(mWaiting, playerTile, maxTiles);

        for (const auto& [changedTile, changeType] : changedTiles)
        {
            if (mPushed.emplace(agentHalfExtents, changedTile).second)
            {
                const auto processTime = changeType == ChangeType::update
                    ? mLastUpdates[std::tie(agentHalfExtents, changedTile)] + mSettings.get().mMinUpdateInterval
                    : std::chrono::steady_clock::time_point();

                const JobIt it = mJobs.emplace(mJobs.end(), agentHalfExtents, navMeshCacheItem, worldspace,
                    changedTile, changeType, getManhattanDistance(changedTile, playerTile), processTime);

                Log(Debug::Debug) << "Post job " << &*it << " for agent=(" << it->mAgentHalfExtents << ")"
                    << " changedTile=(" << it->mChangedTile << ")";

                if (playerTileChanged)
                    mWaiting.push_back(it);
                else
                    insertPrioritizedJob(it, mWaiting);
            }
        }

        if (playerTileChanged)
            std::sort(mWaiting.begin(), mWaiting.end(), LessByJobPriority {});

        Log(Debug::Debug) << "Posted " << mJobs.size() << " navigator jobs";

        if (!mWaiting.empty())
            mHasJob.notify_all();

        lock.unlock();

        if (playerTileChanged && mDbWorker != nullptr)
            mDbWorker->updateJobs(playerTile, maxTiles);
    }

    void AsyncNavMeshUpdater::wait(Loading::Listener& listener, WaitConditionType waitConditionType)
    {
        if (mSettings.get().mWaitUntilMinDistanceToPlayer == 0)
            return;
        listener.setLabel("Building navigation mesh");
        const std::size_t initialJobsLeft = getTotalJobs();
        std::size_t maxProgress = initialJobsLeft + mThreads.size();
        listener.setProgressRange(maxProgress);
        switch (waitConditionType)
        {
            case WaitConditionType::requiredTilesPresent:
            {
                const int minDistanceToPlayer = waitUntilJobsDoneForNotPresentTiles(initialJobsLeft, maxProgress, listener);
                if (minDistanceToPlayer < mSettings.get().mWaitUntilMinDistanceToPlayer)
                {
                    mProcessingTiles.wait(mProcessed, [] (const auto& v) { return v.empty(); });
                    listener.setProgress(maxProgress);
                }
                break;
            }
            case WaitConditionType::allJobsDone:
                waitUntilAllJobsDone();
                listener.setProgress(maxProgress);
                break;
        }
    }

    int AsyncNavMeshUpdater::waitUntilJobsDoneForNotPresentTiles(const std::size_t initialJobsLeft, std::size_t& maxProgress, Loading::Listener& listener)
    {
        std::size_t prevJobsLeft = initialJobsLeft;
        std::size_t jobsDone = 0;
        std::size_t jobsLeft = 0;
        const int maxDistanceToPlayer = mSettings.get().mWaitUntilMinDistanceToPlayer;
        const TilePosition playerPosition = *mPlayerTile.lockConst();
        int minDistanceToPlayer = 0;
        const auto isDone = [&]
        {
            jobsLeft = mJobs.size();
            if (jobsLeft == 0)
            {
                minDistanceToPlayer = 0;
                return true;
            }
            minDistanceToPlayer = getMinDistanceTo(playerPosition, maxDistanceToPlayer, mPushed, mPresentTiles);
            return minDistanceToPlayer >= maxDistanceToPlayer;
        };
        std::unique_lock<std::mutex> lock(mMutex);
        while (!mDone.wait_for(lock, std::chrono::milliseconds(250), isDone))
        {
            if (maxProgress < jobsLeft)
            {
                maxProgress = jobsLeft + mThreads.size();
                listener.setProgressRange(maxProgress);
                listener.setProgress(jobsDone);
            }
            else if (jobsLeft < prevJobsLeft)
            {
                const std::size_t newJobsDone = prevJobsLeft - jobsLeft;
                jobsDone += newJobsDone;
                prevJobsLeft = jobsLeft;
                listener.increaseProgress(newJobsDone);
            }
        }
        return minDistanceToPlayer;
    }

    void AsyncNavMeshUpdater::waitUntilAllJobsDone()
    {
        {
            std::unique_lock<std::mutex> lock(mMutex);
            mDone.wait(lock, [this] { return mJobs.size() == 0; });
        }
        mProcessingTiles.wait(mProcessed, [] (const auto& v) { return v.empty(); });
    }

    AsyncNavMeshUpdater::Stats AsyncNavMeshUpdater::getStats() const
    {
        Stats result;
        {
            const std::lock_guard<std::mutex> lock(mMutex);
            result.mJobs = mJobs.size();
            result.mWaiting = mWaiting.size();
            result.mPushed = mPushed.size();
        }
        result.mProcessing = mProcessingTiles.lockConst()->size();
        if (mDbWorker != nullptr)
            result.mDb = mDbWorker->getStats();
        result.mCache = mNavMeshTilesCache.getStats();
        result.mDbGetTileHits = mDbGetTileHits.load(std::memory_order_relaxed);
        return result;
    }

    void reportStats(const AsyncNavMeshUpdater::Stats& stats, unsigned int frameNumber, osg::Stats& out)
    {
        out.setAttribute(frameNumber, "NavMesh Jobs", static_cast<double>(stats.mJobs));
        out.setAttribute(frameNumber, "NavMesh Waiting", static_cast<double>(stats.mWaiting));
        out.setAttribute(frameNumber, "NavMesh Pushed", static_cast<double>(stats.mPushed));
        out.setAttribute(frameNumber, "NavMesh Processing", static_cast<double>(stats.mProcessing));

        if (stats.mDb.has_value())
        {
            out.setAttribute(frameNumber, "NavMesh DbJobs", static_cast<double>(stats.mDb->mJobs));

            if (stats.mDb->mGetTileCount > 0)
                out.setAttribute(frameNumber, "NavMesh DbCacheHitRate", static_cast<double>(stats.mDbGetTileHits)
                                    / static_cast<double>(stats.mDb->mGetTileCount) * 100.0);
        }

        reportStats(stats.mCache, frameNumber, out);
    }

    void AsyncNavMeshUpdater::process() noexcept
    {
        Log(Debug::Debug) << "Start process navigator jobs by thread=" << std::this_thread::get_id();
        Misc::setCurrentThreadIdlePriority();
        while (!mShouldStop)
        {
            try
            {
                if (JobIt job = getNextJob(); job != mJobs.end())
                {
                    const JobStatus status = processJob(*job);
                    Log(Debug::Debug) << "Processed job " << &*job << " with status=" << status;
                    switch (status)
                    {
                        case JobStatus::Done:
                            unlockTile(job->mAgentHalfExtents, job->mChangedTile);
                            if (job->mGeneratedNavMeshData != nullptr)
                                mDbWorker->addJob(job);
                            else
                                removeJob(job);
                            break;
                        case JobStatus::Fail:
                            repost(job);
                            break;
                        case JobStatus::MemoryCacheMiss:
                        {
                            mDbWorker->addJob(job);
                            break;
                        }
                    }
                }
                else
                    cleanupLastUpdates();
            }
            catch (const std::exception& e)
            {
                Log(Debug::Error) << "AsyncNavMeshUpdater::process exception: " << e.what();
            }
        }
        Log(Debug::Debug) << "Stop navigator jobs processing by thread=" << std::this_thread::get_id();
    }

    JobStatus AsyncNavMeshUpdater::processJob(Job& job)
    {
        Log(Debug::Debug) << "Processing job " << &job << " by thread=" << std::this_thread::get_id();

        const auto navMeshCacheItem = job.mNavMeshCacheItem.lock();

        if (!navMeshCacheItem)
            return JobStatus::Done;

        const auto playerTile = *mPlayerTile.lockConst();
        const auto params = *navMeshCacheItem->lockConst()->getImpl().getParams();

        if (!shouldAddTile(job.mChangedTile, playerTile, std::min(mSettings.get().mMaxTilesNumber, params.maxTiles)))
        {
            Log(Debug::Debug) << "Ignore add tile by job " << &job << ": too far from player";
            navMeshCacheItem->lock()->removeTile(job.mChangedTile);
            return JobStatus::Done;
        }

        switch (job.mState)
        {
            case JobState::Initial:
                return processInitialJob(job, *navMeshCacheItem);
            case JobState::WithDbResult:
                return processJobWithDbResult(job, *navMeshCacheItem);
        }

        return JobStatus::Done;
    }

    JobStatus AsyncNavMeshUpdater::processInitialJob(Job& job, GuardedNavMeshCacheItem& navMeshCacheItem)
    {
        Log(Debug::Debug) << "Processing initial job " << &job;

        std::shared_ptr<RecastMesh> recastMesh = mRecastMeshManager.get().getMesh(job.mWorldspace, job.mChangedTile);

        if (recastMesh == nullptr)
        {
            Log(Debug::Debug) << "Null recast mesh for job " << &job;
            navMeshCacheItem.lock()->markAsEmpty(job.mChangedTile);
            return JobStatus::Done;
        }

        const auto bounds = getBounds(*recastMesh, job.mAgentHalfExtents, mSettings);

        if (isEmpty(bounds))
        {
            Log(Debug::Debug) << "Empty bounds for job " << &job;
            navMeshCacheItem.lock()->markAsEmpty(job.mChangedTile);
            return JobStatus::Done;
        }

        NavMeshTilesCache::Value cachedNavMeshData = mNavMeshTilesCache.get(job.mAgentHalfExtents, job.mChangedTile, *recastMesh);
        std::unique_ptr<PreparedNavMeshData> preparedNavMeshData;
        const PreparedNavMeshData* preparedNavMeshDataPtr = nullptr;

        if (cachedNavMeshData)
        {
            preparedNavMeshDataPtr = &cachedNavMeshData.get();
        }
        else
        {
            if (job.mChangeType != ChangeType::update && mDbWorker != nullptr)
            {
                job.mConfig = makeRecastConfig(job.mChangedTile, bounds, job.mAgentHalfExtents, mSettings);
                job.mBounds = bounds;
                job.mInput = serialize(mSettings.get().mRecastScaleFactor, job.mConfig, *recastMesh);
                job.mRecastMesh = std::move(recastMesh);
                return JobStatus::MemoryCacheMiss;
            }

            const rcConfig config = makeRecastConfig(job.mChangedTile, bounds, job.mAgentHalfExtents, mSettings);
            preparedNavMeshData = prepareNavMeshTileData(config, *recastMesh, job.mAgentHalfExtents, mSettings);

            if (preparedNavMeshData == nullptr)
            {
                Log(Debug::Debug) << "Null navmesh data for job " << &job;
                navMeshCacheItem.lock()->markAsEmpty(job.mChangedTile);
                return JobStatus::Done;
            }

            if (job.mChangeType == ChangeType::update)
            {
                preparedNavMeshDataPtr = preparedNavMeshData.get();
            }
            else
            {
                cachedNavMeshData = mNavMeshTilesCache.set(job.mAgentHalfExtents, job.mChangedTile,
                                                           *recastMesh, std::move(preparedNavMeshData));
                preparedNavMeshDataPtr = cachedNavMeshData ? &cachedNavMeshData.get() : preparedNavMeshData.get();
            }
        }

        const auto offMeshConnections = mOffMeshConnectionsManager.get().get(job.mChangedTile);

        const UpdateNavMeshStatus status = navMeshCacheItem.lock()->updateTile(job.mChangedTile, std::move(cachedNavMeshData),
            makeNavMeshTileData(*preparedNavMeshDataPtr, offMeshConnections, job.mAgentHalfExtents, job.mChangedTile, mSettings));

        return handleUpdateNavMeshStatus(status, job, navMeshCacheItem, *recastMesh);
    }

    JobStatus AsyncNavMeshUpdater::processJobWithDbResult(Job& job, GuardedNavMeshCacheItem& navMeshCacheItem)
    {
        Log(Debug::Debug) << "Processing job with db result " << &job;

        std::unique_ptr<PreparedNavMeshData> preparedNavMeshData;
        bool generatedNavMeshData = false;

        if (job.mCachedTileData.has_value() && job.mCachedTileData->mVersion == mSettings.get().mNavMeshVersion)
        {
            preparedNavMeshData = std::make_unique<PreparedNavMeshData>();
            if (deserialize(job.mCachedTileData->mData, *preparedNavMeshData))
                ++mDbGetTileHits;
            else
                preparedNavMeshData = nullptr;
        }

        if (preparedNavMeshData == nullptr)
        {
            preparedNavMeshData = prepareNavMeshTileData(job.mConfig, *job.mRecastMesh, job.mAgentHalfExtents, mSettings);
            generatedNavMeshData = true;
        }

        if (preparedNavMeshData == nullptr)
        {
            Log(Debug::Debug) << "Null navmesh data for job " << &job;
            navMeshCacheItem.lock()->markAsEmpty(job.mChangedTile);
            return JobStatus::Done;
        }

        auto cachedNavMeshData = mNavMeshTilesCache.set(job.mAgentHalfExtents, job.mChangedTile, *job.mRecastMesh,
                                                        std::move(preparedNavMeshData));

        const auto offMeshConnections = mOffMeshConnectionsManager.get().get(job.mChangedTile);

        const PreparedNavMeshData* preparedNavMeshDataPtr = cachedNavMeshData ? &cachedNavMeshData.get() : preparedNavMeshData.get();
        const UpdateNavMeshStatus status = navMeshCacheItem.lock()->updateTile(job.mChangedTile, std::move(cachedNavMeshData),
            makeNavMeshTileData(*preparedNavMeshDataPtr, offMeshConnections, job.mAgentHalfExtents, job.mChangedTile, mSettings));

        const JobStatus result = handleUpdateNavMeshStatus(status, job, navMeshCacheItem, *job.mRecastMesh);

        if (result == JobStatus::Done && job.mChangeType != ChangeType::update
                && mDbWorker != nullptr && mSettings.get().mWriteToNavMeshDb && generatedNavMeshData)
            job.mGeneratedNavMeshData = std::make_unique<PreparedNavMeshData>(*preparedNavMeshDataPtr);

        return result;
    }

    JobStatus AsyncNavMeshUpdater::handleUpdateNavMeshStatus(UpdateNavMeshStatus status,
        const Job& job, const GuardedNavMeshCacheItem& navMeshCacheItem, const RecastMesh& recastMesh)
    {
        Version navMeshVersion;
        {
            const auto locked = navMeshCacheItem.lockConst();
            navMeshVersion.mGeneration = locked->getGeneration();
            navMeshVersion.mRevision = locked->getNavMeshRevision();
        }
        mRecastMeshManager.get().reportNavMeshChange(job.mChangedTile,
            Version {recastMesh.getGeneration(), recastMesh.getRevision()},
            navMeshVersion);

        if (status == UpdateNavMeshStatus::removed || status == UpdateNavMeshStatus::lost)
        {
            const std::scoped_lock lock(mMutex);
            mPresentTiles.erase(std::make_tuple(job.mAgentHalfExtents, job.mChangedTile));
        }
        else if (isSuccess(status) && status != UpdateNavMeshStatus::ignored)
        {
            const std::scoped_lock lock(mMutex);
            mPresentTiles.insert(std::make_tuple(job.mAgentHalfExtents, job.mChangedTile));
        }

        writeDebugFiles(job, &recastMesh);

        return isSuccess(status) ? JobStatus::Done : JobStatus::Fail;
    }

    JobIt AsyncNavMeshUpdater::getNextJob()
    {
        std::unique_lock<std::mutex> lock(mMutex);

        bool shouldStop = false;
        const auto hasJob = [&]
        {
            shouldStop = mShouldStop;
            return shouldStop
                || (!mWaiting.empty() && mWaiting.front()->mProcessTime <= std::chrono::steady_clock::now());
        };

        if (!mHasJob.wait_for(lock, std::chrono::milliseconds(10), hasJob))
        {
            if (mJobs.empty())
                mDone.notify_all();
            return mJobs.end();
        }

        if (shouldStop)
            return mJobs.end();

        const JobIt job = mWaiting.front();

        mWaiting.pop_front();

        if (job->mRecastMesh != nullptr)
            return job;

        if (!lockTile(job->mAgentHalfExtents, job->mChangedTile))
        {
            Log(Debug::Debug) << "Failed to lock tile by " << &*job;
            ++job->mTryNumber;
            insertPrioritizedJob(job, mWaiting);
            return mJobs.end();
        }

        if (job->mChangeType == ChangeType::update)
            mLastUpdates[getAgentAndTile(*job)] = std::chrono::steady_clock::now();
        mPushed.erase(getAgentAndTile(*job));

        return job;
    }

    void AsyncNavMeshUpdater::writeDebugFiles(const Job& job, const RecastMesh* recastMesh) const
    {
        std::string revision;
        std::string recastMeshRevision;
        std::string navMeshRevision;
        if ((mSettings.get().mEnableWriteNavMeshToFile || mSettings.get().mEnableWriteRecastMeshToFile)
                && (mSettings.get().mEnableRecastMeshFileNameRevision || mSettings.get().mEnableNavMeshFileNameRevision))
        {
            revision = "." + std::to_string((std::chrono::steady_clock::now()
                - std::chrono::steady_clock::time_point()).count());
            if (mSettings.get().mEnableRecastMeshFileNameRevision)
                recastMeshRevision = revision;
            if (mSettings.get().mEnableNavMeshFileNameRevision)
                navMeshRevision = revision;
        }
        if (recastMesh && mSettings.get().mEnableWriteRecastMeshToFile)
            writeToFile(*recastMesh, mSettings.get().mRecastMeshPathPrefix + std::to_string(job.mChangedTile.x())
                        + "_" + std::to_string(job.mChangedTile.y()) + "_", recastMeshRevision);
        if (mSettings.get().mEnableWriteNavMeshToFile)
            if (const auto shared = job.mNavMeshCacheItem.lock())
                writeToFile(shared->lockConst()->getImpl(), mSettings.get().mNavMeshPathPrefix, navMeshRevision);
    }

    void AsyncNavMeshUpdater::repost(JobIt job)
    {
        unlockTile(job->mAgentHalfExtents, job->mChangedTile);

        if (mShouldStop || job->mTryNumber > 2)
            return;

        const std::lock_guard<std::mutex> lock(mMutex);

        if (mPushed.emplace(job->mAgentHalfExtents, job->mChangedTile).second)
        {
            ++job->mTryNumber;
            insertPrioritizedJob(job, mWaiting);
            mHasJob.notify_all();
            return;
        }

        mJobs.erase(job);
    }

    bool AsyncNavMeshUpdater::lockTile(const osg::Vec3f& agentHalfExtents, const TilePosition& changedTile)
    {
        Log(Debug::Debug) << "Locking tile agent=(" << agentHalfExtents << ") changedTile=(" << changedTile << ")";
        return mProcessingTiles.lock()->emplace(agentHalfExtents, changedTile).second;
    }

    void AsyncNavMeshUpdater::unlockTile(const osg::Vec3f& agentHalfExtents, const TilePosition& changedTile)
    {
        auto locked = mProcessingTiles.lock();
        locked->erase(std::tie(agentHalfExtents, changedTile));
        Log(Debug::Debug) << "Unlocked tile agent=(" << agentHalfExtents << ") changedTile=(" << changedTile << ")";
        if (locked->empty())
            mProcessed.notify_all();
    }

    std::size_t AsyncNavMeshUpdater::getTotalJobs() const
    {
        const std::scoped_lock lock(mMutex);
        return mJobs.size();
    }

    void AsyncNavMeshUpdater::cleanupLastUpdates()
    {
        const auto now = std::chrono::steady_clock::now();

        const std::lock_guard<std::mutex> lock(mMutex);

        for (auto it = mLastUpdates.begin(); it != mLastUpdates.end();)
        {
            if (now - it->second > mSettings.get().mMinUpdateInterval)
                it = mLastUpdates.erase(it);
            else
                ++it;
        }
    }

    void AsyncNavMeshUpdater::addJob(JobIt job)
    {
        const std::lock_guard lock(mMutex);
        insertPrioritizedJob(job, mWaiting);
        mHasJob.notify_all();
    }

    void AsyncNavMeshUpdater::removeJob(JobIt job)
    {
        Log(Debug::Debug) << "Removing job " << &*job;
        const std::lock_guard lock(mMutex);
        mJobs.erase(job);
    }

    void DbJobQueue::push(JobIt job)
    {
        const std::lock_guard lock(mMutex);
        insertPrioritizedDbJob(job, mJobs);
        mHasJob.notify_all();
    }

    std::optional<JobIt> DbJobQueue::pop()
    {
        std::unique_lock lock(mMutex);
        mHasJob.wait(lock, [&] { return mShouldStop || !mJobs.empty(); });
        if (mJobs.empty())
            return std::nullopt;
        const JobIt job = mJobs.front();
        mJobs.pop_front();
        return job;
    }

    void DbJobQueue::update(TilePosition playerTile, int maxTiles)
    {
        const std::lock_guard lock(mMutex);
        updateJobs(mJobs, playerTile, maxTiles);
        std::sort(mJobs.begin(), mJobs.end(), LessByJobDbPriority {});
    }

    void DbJobQueue::stop()
    {
        const std::lock_guard lock(mMutex);
        mJobs.clear();
        mShouldStop = true;
        mHasJob.notify_all();
    }

    std::size_t DbJobQueue::size() const
    {
        const std::lock_guard lock(mMutex);
        return mJobs.size();
    }

    DbWorker::DbWorker(AsyncNavMeshUpdater& updater, std::unique_ptr<NavMeshDb>&& db, TileVersion version)
        : mUpdater(updater)
        , mDb(std::move(db))
        , mVersion(version)
        , mNextTileId(mDb->getMaxTileId() + 1)
        , mThread([this] { run(); })
    {
    }

    DbWorker::~DbWorker()
    {
        stop();
        mThread.join();
    }

    DbWorker::Stats DbWorker::getStats() const
    {
        Stats result;
        result.mJobs = mQueue.size();
        result.mGetTileCount = mGetTileCount.load(std::memory_order::memory_order_relaxed);
        return result;
    }

    void DbWorker::stop()
    {
        mShouldStop = true;
        mQueue.stop();
    }

    void DbWorker::run() noexcept
    {
        constexpr std::size_t writesPerTransaction = 100;
        auto transaction = mDb->startTransaction();
        while (!mShouldStop)
        {
            try
            {
                if (const auto job = mQueue.pop())
                    processJob(*job);
            }
            catch (const std::exception& e)
            {
                Log(Debug::Error) << "DbWorker exception: " << e.what();
            }
            if (mWrites > writesPerTransaction)
            {
                mWrites = 0;
                transaction.commit();
                transaction = mDb->startTransaction();
            }
        }
        transaction.commit();
    }

    void DbWorker::processJob(JobIt job)
    {
        RemoveJob removeJob(mUpdater, job);

        if (const auto& generatedTileData = job->mGeneratedNavMeshData)
        {
            ++mWrites;
            Log(Debug::Debug) << "Processing db write job " << &*job;
            if (const auto& cachedTileData = job->mCachedTileData)
            {
                Log(Debug::Debug) << "Update db tile by job " << &*job;
                generatedTileData->mUserId = cachedTileData->mTileId;
                mDb->updateTile(cachedTileData->mTileId, mVersion, serialize(*generatedTileData));
                return;
            }
            const auto cached = mDb->findTile(job->mWorldspace, job->mChangedTile, job->mInput);
            if (cached.has_value() && cached->mVersion == mVersion)
            {
                Log(Debug::Debug) << "Ignore existing db tile by job " << &*job;
                return;
            }
            generatedTileData->mUserId = mNextTileId;
            Log(Debug::Debug) << "Insert db tile by job " << &*job;
            mDb->insertTile(mNextTileId, job->mWorldspace, job->mChangedTile,
                            mVersion, job->mInput, serialize(*generatedTileData));
            ++mNextTileId.t;
            return;
        }

        Log(Debug::Debug) << "Processing db read job " << &*job;
        job->mCachedTileData = mDb->getTileData(job->mWorldspace, job->mChangedTile, job->mInput);
        job->mState = JobState::WithDbResult;
        ++mGetTileCount;
        mUpdater.addJob(job);
        removeJob.cancel();
    }
}
