#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <LinearMath/btThreads.h>

#include "components/debug/debuglog.hpp"
#include "components/misc/convert.hpp"
#include "components/settings/settings.hpp"
#include "../mwmechanics/actorutil.hpp"
#include "../mwmechanics/movement.hpp"
#include "../mwworld/class.hpp"
#include "../mwworld/player.hpp"

#include "actor.hpp"
#include "movementsolver.hpp"
#include "mtphysics.hpp"
#include "object.hpp"
#include "physicssystem.hpp"

class btIParallelSumBody; // needed to compile with bullet < 2.88

namespace
{
    /// @brief A scoped lock that is either shared or exclusive depending on configuration
    template<class Mutex>
    class MaybeSharedLock
    {
        public:
            /// @param mutex a shared mutex
            /// @param canBeSharedLock decide wether the lock will be shared or exclusive
            MaybeSharedLock(Mutex& mutex, bool canBeSharedLock) : mMutex(mutex), mCanBeSharedLock(canBeSharedLock)
            {
                if (mCanBeSharedLock)
                    mMutex.lock_shared();
                else
                    mMutex.lock();
            }

            ~MaybeSharedLock()
            {
                if (mCanBeSharedLock)
                    mMutex.unlock_shared();
                else
                    mMutex.unlock();
            }
        private:
            Mutex& mMutex;
            bool mCanBeSharedLock;
    };

    void handleFall(const MWPhysics::ActorFrameData& actorData, bool simulationPerformed)
    {
        const MWWorld::Ptr player = MWMechanics::getPlayer();
        const float heightDiff = actorData.mPosition.z() - actorData.mOldHeight;

        const auto& character = actorData.mPtr;

        MWMechanics::CreatureStats& stats = character.getClass().getCreatureStats(character);

        const bool isStillOnGround = (simulationPerformed && actorData.mWasOnGround && actorData.mActorRaw->getOnGround());

        if (isStillOnGround || actorData.mFlying || actorData.mSwimming || actorData.mSlowFall < 1)
            stats.land(character == player && (actorData.mFlying || actorData.mSwimming));
        else if (heightDiff < 0)
            stats.addToFallHeight(-heightDiff);
    }

    void handleJump(const MWWorld::Ptr &ptr)
    {
        const bool isPlayer = (ptr == MWMechanics::getPlayer());
        // Advance acrobatics and set flag for GetPCJumping
        if (isPlayer)
        {
            ptr.getClass().skillUsageSucceeded(ptr, ESM::Skill::Acrobatics, 0);
            MWBase::Environment::get().getWorld()->getPlayer().setJumping(true);
        }

        // Decrease fatigue
        if (!isPlayer || !MWBase::Environment::get().getWorld()->getGodModeState())
        {
            const MWWorld::Store<ESM::GameSetting> &gmst = MWBase::Environment::get().getWorld()->getStore().get<ESM::GameSetting>();
            const float fFatigueJumpBase = gmst.find("fFatigueJumpBase")->mValue.getFloat();
            const float fFatigueJumpMult = gmst.find("fFatigueJumpMult")->mValue.getFloat();
            const float normalizedEncumbrance = std::min(1.f, ptr.getClass().getNormalizedEncumbrance(ptr));
            const float fatigueDecrease = fFatigueJumpBase + normalizedEncumbrance * fFatigueJumpMult;
            MWMechanics::DynamicStat<float> fatigue = ptr.getClass().getCreatureStats(ptr).getFatigue();
            fatigue.setCurrent(fatigue.getCurrent() - fatigueDecrease);
            ptr.getClass().getCreatureStats(ptr).setFatigue(fatigue);
        }
        ptr.getClass().getMovementSettings(ptr).mPosition[2] = 0;
    }

    osg::Vec3f interpolateMovements(const MWPhysics::ActorFrameData& actorData, float timeAccum, float physicsDt)
    {
        const float interpolationFactor = timeAccum / physicsDt;
        return actorData.mPosition * interpolationFactor + actorData.mActorRaw->getPreviousPosition() * (1.f - interpolationFactor);
    }

    namespace Config
    {
        /* The purpose of these 2 classes is to make OpenMW works with Bullet compiled with either single or multithread support.
           At runtime, Bullet resolve the call to btParallelFor() to:
           - btITaskScheduler::parallelFor() if bullet is multithreaded
           - btIParallelForBody::forLoop() if bullet is singlethreaded.

           NOTE: From Bullet 2.88, there is a btDefaultTaskScheduler(), that returns NULL if multithreading is not supported.
           It might be worth considering to simplify the API once OpenMW stops supporting 2.87.
        */

        class MultiThreadedBullet : public btITaskScheduler
        {
            public:
                MultiThreadedBullet(): btITaskScheduler("") {};
                ~MultiThreadedBullet() override = default;
                int getMaxNumThreads() const override { return 1; };
                int getNumThreads() const override { return 1; };
                void setNumThreads(int numThreads) override {};

                /// @brief will be called by Bullet if threading is supported
                void parallelFor(int iBegin, int iEnd, int batchsize, const btIParallelForBody& body) override {};

                /// @brief stub implementation needed to compile with bullet >= 2.88. Method doesn't exists in < 2.87 so we can't use override keyword
                btScalar parallelSum(int iBegin, int iEnd, int grainSize, const btIParallelSumBody& body) { return {}; };
        };

        class SingleThreadedBullet : public btIParallelForBody
        {
            public:
                explicit SingleThreadedBullet(bool &threadingSupported): mThreadingSupported(threadingSupported) {};
                /// @brief will be called by Bullet if threading is NOT supported
                void forLoop(int iBegin, int iEnd) const override
                {
                    mThreadingSupported = false;
                }
            private:
                bool &mThreadingSupported;
        };

        /// @return either the number of thread as configured by the user, or 1 if Bullet doesn't support multithreading
        int computeNumThreads(bool& threadSafeBullet)
        {
            int wantedThread = Settings::Manager::getInt("async num threads", "Physics");

            auto bulletScheduler = std::make_unique<MultiThreadedBullet>();
            btSetTaskScheduler(bulletScheduler.get());
            bool threadingSupported = true;
            btParallelFor(0, 0, 0, SingleThreadedBullet(threadingSupported));

            threadSafeBullet = threadingSupported;
            if (!threadingSupported && wantedThread > 1)
            {
                Log(Debug::Warning) << "Bullet was not compiled with multithreading support, 1 async thread will be used";
                return 1;
            }
            return std::max(0, wantedThread);
        }
    }
}

namespace MWPhysics
{
    PhysicsTaskScheduler::PhysicsTaskScheduler(float physicsDt, std::shared_ptr<btCollisionWorld> collisionWorld)
          : mPhysicsDt(physicsDt)
          , mCollisionWorld(collisionWorld)
          , mNumJobs(0)
          , mRemainingSteps(0)
          , mLOSCacheExpiry(Settings::Manager::getInt("lineofsight keep inactive cache", "Physics"))
          , mDeferAabbUpdate(Settings::Manager::getBool("defer aabb update", "Physics"))
          , mNewFrame(false)
          , mNextJob(0)
          , mQuit(false)
    {
        mNumThreads = Config::computeNumThreads(mThreadSafeBullet);

        if (mNumThreads >= 1)
        {
            for (int i = 0; i < mNumThreads; ++i)
                mThreads.emplace_back([&] { worker(); } );
        }
        else
        {
            mLOSCacheExpiry = -1;
            mDeferAabbUpdate = false;
        }

        mPreStepBarrier = std::make_unique<Barrier>(mNumThreads);
        mPostStepBarrier = std::make_unique<Barrier>(mNumThreads);
        mPostSimBarrier = std::make_unique<Barrier>(mNumThreads);
    }

    PhysicsTaskScheduler::~PhysicsTaskScheduler()
    {
        mQuit = true;
        mHasJob.notify_all();
        for (auto& thread : mThreads)
            thread.join();
        mActorsFrameData.clear();
    }

    const PtrPositionList& PhysicsTaskScheduler::moveActors(int numSteps, float timeAccum, std::vector<ActorFrameData>&& actorsData, CollisionMap& standingCollisions, WorldFrameData worldData)
    {
        std::lock_guard<std::shared_timed_mutex> lock(mSimulationMutex);

        for (const auto& data : mActorsFrameData)
        {
            // Remove actors that were deleted while the background thread was running
            if (!data.mActor.lock())
            {
                mMovementResults.erase(data.mPtr);
                continue;
            }
            if (data.mDidJump)
                handleJump(data.mPtr);
        }

        std::swap(mMovementResults, mPreviousMovementResults);
        std::swap(standingCollisions, mStandingCollisions);

        mRemainingSteps = numSteps;
        mTimeAccum = timeAccum;
        mWorldFrameData = worldData;
        mActorsFrameData = std::move(actorsData);

        mAdvanceSimulation = (mRemainingSteps != 0);

        mMovementResults.clear();
        for (auto const& m : mActorsFrameData)
            mMovementResults[m.mPtr] = m.mPosition;

        if (mNumThreads == 0)
            syncComputation();
        else
            asyncComputation();

        return mPreviousMovementResults;
    }

    void PhysicsTaskScheduler::rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld, btCollisionWorld::RayResultCallback& resultCallback) const
    {
        MaybeSharedLock<std::shared_timed_mutex> lock(mCollisionWorldMutex, mThreadSafeBullet);
        mCollisionWorld->rayTest(rayFromWorld, rayToWorld, resultCallback);
    }

    void PhysicsTaskScheduler::convexSweepTest(const btConvexShape* castShape, const btTransform& from, const btTransform& to, btCollisionWorld::ConvexResultCallback& resultCallback) const
    {
        MaybeSharedLock<std::shared_timed_mutex> lock(mCollisionWorldMutex, mThreadSafeBullet);
        mCollisionWorld->convexSweepTest(castShape, from, to, resultCallback);
    }

    void PhysicsTaskScheduler::contactTest(btCollisionObject* colObj, btCollisionWorld::ContactResultCallback& resultCallback)
    {
        std::shared_lock<std::shared_timed_mutex> lock(mCollisionWorldMutex);
        mCollisionWorld->contactTest(colObj, resultCallback);
    }

    boost::optional<btVector3> PhysicsTaskScheduler::getHitPoint(const btTransform& from, btCollisionObject* target)
    {
        MaybeSharedLock<std::shared_timed_mutex> lock(mCollisionWorldMutex, mThreadSafeBullet);
        // target the collision object's world origin, this should be the center of the collision object
        btTransform rayTo;
        rayTo.setIdentity();
        rayTo.setOrigin(target->getWorldTransform().getOrigin());

        btCollisionWorld::ClosestRayResultCallback cb(from.getOrigin(), rayTo.getOrigin());

        mCollisionWorld->rayTestSingle(from, rayTo, target, target->getCollisionShape(), target->getWorldTransform(), cb);
        if (!cb.hasHit())
            // didn't hit the target. this could happen if point is already inside the collision box
            return boost::none;
        return {cb.m_hitPointWorld};
    }

    void PhysicsTaskScheduler::aabbTest(const btVector3& aabbMin, const btVector3& aabbMax, btBroadphaseAabbCallback& callback)
    {
        std::shared_lock<std::shared_timed_mutex> lock(mCollisionWorldMutex);
        mCollisionWorld->getBroadphase()->aabbTest(aabbMin, aabbMax, callback);
    }

    void PhysicsTaskScheduler::getAabb(const btCollisionObject* obj, btVector3& min, btVector3& max)
    {
        std::shared_lock<std::shared_timed_mutex> lock(mCollisionWorldMutex);
        obj->getCollisionShape()->getAabb(obj->getWorldTransform(), min, max);
    }

    void PhysicsTaskScheduler::setCollisionFilterMask(btCollisionObject* collisionObject, int collisionFilterMask)
    {
        std::unique_lock<std::shared_timed_mutex> lock(mCollisionWorldMutex);
        collisionObject->getBroadphaseHandle()->m_collisionFilterMask = collisionFilterMask;
    }

    void PhysicsTaskScheduler::addCollisionObject(btCollisionObject* collisionObject, int collisionFilterGroup, int collisionFilterMask)
    {
        std::unique_lock<std::shared_timed_mutex> lock(mCollisionWorldMutex);
        mCollisionWorld->addCollisionObject(collisionObject, collisionFilterGroup, collisionFilterMask);
    }

    void PhysicsTaskScheduler::removeCollisionObject(btCollisionObject* collisionObject)
    {
        std::unique_lock<std::shared_timed_mutex> lock(mCollisionWorldMutex);
        mCollisionWorld->removeCollisionObject(collisionObject);
    }

    void PhysicsTaskScheduler::updateSingleAabb(std::weak_ptr<PtrHolder> ptr)
    {
        if (mDeferAabbUpdate)
        {
            std::unique_lock<std::mutex> lock(mUpdateAabbMutex);
            mUpdateAabb.insert(std::move(ptr));
        }
        else
        {
            std::unique_lock<std::shared_timed_mutex> lock(mCollisionWorldMutex);
            if (auto p = ptr.lock())
            {
                if (auto actor = std::dynamic_pointer_cast<Actor>(p))
                {
                    actor->commitPositionChange();
                    mCollisionWorld->updateSingleAabb(actor->getCollisionObject());
                }
                else if (auto object = std::dynamic_pointer_cast<Object>(p))
                {
                    object->commitPositionChange();
                    mCollisionWorld->updateSingleAabb(object->getCollisionObject());
                }
            }
        }
    }

    bool PhysicsTaskScheduler::getLineOfSight(const std::weak_ptr<Actor>& actor1, const std::weak_ptr<Actor>& actor2)
    {
        std::unique_lock<std::shared_timed_mutex> lock(mLOSCacheMutex);

        auto actorPtr1 = actor1.lock();
        auto actorPtr2 = actor2.lock();
        if (!actorPtr1 || !actorPtr2)
            return false;

        auto req = LOSRequest(actor1, actor2);
        auto result = std::find(mLOSCache.begin(), mLOSCache.end(), req);
        if (result == mLOSCache.end())
        {
            req.mResult = hasLineOfSight(actorPtr1.get(), actorPtr2.get());
            if (mLOSCacheExpiry >= 0)
                mLOSCache.push_back(req);
        }
        req.mAge = 0;
        return req.mResult;
    }

    void PhysicsTaskScheduler::refreshLOSCache()
    {
        std::shared_lock<std::shared_timed_mutex> lock(mLOSCacheMutex);
        int job = 0;
        int numLOS = mLOSCache.size();
        while ((job = mNextLOS.fetch_add(1, std::memory_order_relaxed)) < numLOS)
        {
            auto& req = mLOSCache[job];
            auto actorPtr1 = req.mActors[0].lock();
            auto actorPtr2 = req.mActors[1].lock();

            if (req.mAge++ > mLOSCacheExpiry || !actorPtr1 || !actorPtr2)
                req.mStale = true;
            else
                req.mResult = hasLineOfSight(actorPtr1.get(), actorPtr2.get());
        }

    }

    void PhysicsTaskScheduler::updateAabbs()
    {
        std::unique_lock<std::shared_timed_mutex> lock1(mCollisionWorldMutex, std::defer_lock);
        std::unique_lock<std::mutex> lock2(mUpdateAabbMutex, std::defer_lock);
        std::lock(lock1, lock2);
        std::for_each(mUpdateAabb.begin(), mUpdateAabb.end(),
            [&](std::weak_ptr<PtrHolder> ptr) {
                if (auto p = ptr.lock())
                {
                    if (auto actor = std::dynamic_pointer_cast<Actor>(p))
                    {
                        actor->commitPositionChange();
                        mCollisionWorld->updateSingleAabb(actor->getCollisionObject());
                    }
                    else if (auto object = std::dynamic_pointer_cast<Object>(p))
                    {
                        object->commitPositionChange();
                        mCollisionWorld->updateSingleAabb(object->getCollisionObject());
                    }
                }});
        mUpdateAabb.clear();
    }

    void PhysicsTaskScheduler::worker()
    {
        std::shared_lock<std::shared_timed_mutex> lock(mSimulationMutex);
        while (!mQuit)
        {
            if (!mNewFrame)
                mHasJob.wait(lock, [&]() { return mQuit || mNewFrame; });

            if (mDeferAabbUpdate)
                mPreStepBarrier->wait([&]() { updateAabbs(); });

            int job = 0;
            while ((job = mNextJob.fetch_add(1, std::memory_order_relaxed)) < mNumJobs)
            {
                MaybeSharedLock<std::shared_timed_mutex> lockColWorld(mCollisionWorldMutex, mThreadSafeBullet);
                if(const auto actor = mActorsFrameData[job].mActor.lock())
                {
                    if (mRemainingSteps)
                        MovementSolver::move(mActorsFrameData[job], mPhysicsDt, mCollisionWorld.get(), mStandingCollisions, mWorldFrameData);
                    else
                    {
                        const auto& actorData = mActorsFrameData[job];
                        handleFall(actorData, mAdvanceSimulation);
                        mMovementResults[actorData.mPtr] = interpolateMovements(actorData, mTimeAccum, mPhysicsDt);
                    }
                }
            }

            const auto oncePerStep = [&]()
            {
                mNextJob.store(0, std::memory_order_release);
                perSimulationStepUpdate();
                if (!mRemainingSteps--)
                {
                    perFrameUpdate();
                    mNewFrame = false;
                }
            };

            mPostStepBarrier->wait(oncePerStep);

            if (!mRemainingSteps && mLOSCacheExpiry >= 0)
            {
                refreshLOSCache();
                const auto cleanCache = [&]()
                {
                    std::unique_lock<std::shared_timed_mutex> lock(mLOSCacheMutex);
                    mLOSCache.erase(
                            std::remove_if(mLOSCache.begin(), mLOSCache.end(),
                                [](const LOSRequest& req){ return req.mStale; }),
                            mLOSCache.end());
                };

                mPostSimBarrier->wait(cleanCache);
            }
        }
    }

    void PhysicsTaskScheduler::perSimulationStepUpdate()
    {
        std::unique_lock<std::shared_timed_mutex> lock(mCollisionWorldMutex);
        for (auto& actorData : mActorsFrameData)
        {
            if(const auto actor = actorData.mActor.lock())
            {
                if (actorData.mPosition == actor->getPosition())
                    actor->setPosition(actorData.mPosition, false); // update previous position to make sure interpolation is correct
                else
                {
                    actorData.mPositionChanged = true;
                    actor->setPosition(actorData.mPosition);
                }
            }
        }
    }

    void PhysicsTaskScheduler::perFrameUpdate()
    {
        std::unique_lock<std::shared_timed_mutex> lock(mCollisionWorldMutex);
        for (const auto& actorData : mActorsFrameData)
            if (actorData.mPositionChanged)
            {
                if(const auto actor = actorData.mActor.lock())
                    mCollisionWorld->updateSingleAabb(actor->getCollisionObject());
            }
    }

    bool PhysicsTaskScheduler::hasLineOfSight(const Actor* actor1, const Actor* actor2)
    {
        btVector3 pos1  = Misc::Convert::toBullet(actor1->getCollisionObjectPosition() + osg::Vec3f(0,0,actor1->getHalfExtents().z() * 0.9)); // eye level
        btVector3 pos2  = Misc::Convert::toBullet(actor2->getCollisionObjectPosition() + osg::Vec3f(0,0,actor2->getHalfExtents().z() * 0.9));

        btCollisionWorld::ClosestRayResultCallback resultCallback(pos1, pos2);
        resultCallback.m_collisionFilterGroup = 0xFF;
        resultCallback.m_collisionFilterMask = CollisionType_World|CollisionType_HeightMap|CollisionType_Door;

        MaybeSharedLock<std::shared_timed_mutex> lockColWorld(mCollisionWorldMutex, mThreadSafeBullet);
        mCollisionWorld->rayTest(pos1, pos2, resultCallback);

        return !resultCallback.hasHit();
    }

    void PhysicsTaskScheduler::syncComputation()
    {
        while (mRemainingSteps--)
        {
            for (auto& actorData : mActorsFrameData)
                MovementSolver::move(actorData, mPhysicsDt, mCollisionWorld.get(), mStandingCollisions, mWorldFrameData);

            perSimulationStepUpdate();
        }

        for (const auto& actorData : mActorsFrameData)
        {
            handleFall(actorData, mAdvanceSimulation);
            mMovementResults[actorData.mPtr] = interpolateMovements(actorData, mTimeAccum, mPhysicsDt);
        }
        perFrameUpdate();

        refreshLOSCache();
    }

    void PhysicsTaskScheduler::asyncComputation()
    {
        mNewFrame = true;
        mNumJobs = mActorsFrameData.size();
        mNextJob.store(0, std::memory_order_release);
        mNextLOS.store(0, std::memory_order_release);
        mHasJob.notify_all();
    }

    PhysicsTaskScheduler::Barrier::Barrier(int count) : mThreadCount(count), mRendezvousCount(0), mGeneration(0) {}

    template<typename F>
    void PhysicsTaskScheduler::Barrier::wait(F&& func)
    {
        std::unique_lock<std::mutex> lock(mMutex);

        ++mRendezvousCount;
        const int currentGeneration = mGeneration;
        if (mRendezvousCount == mThreadCount)
        {
            ++mGeneration;
            mRendezvousCount = 0;
            func();
            mRendezvous.notify_all();
        }
        else
        {
            mRendezvous.wait(lock, [&]() { return mGeneration != currentGeneration; });
        }
    }
}
