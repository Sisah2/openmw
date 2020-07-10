#ifndef OPENMW_MWPHYSICS_MTPHYSICS_H
#define OPENMW_MWPHYSICS_MTPHYSICS_H

#include <atomic>
#include <condition_variable>
#include <thread>
#include <shared_mutex>

#include <boost/optional/optional.hpp>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

#include "physicssystem.hpp"
#include "ptrholder.hpp"

namespace MWPhysics
{
    class PhysicsTaskScheduler
    {
        public:
            PhysicsTaskScheduler(float physicsDt, std::shared_ptr<btCollisionWorld> collisionWorld);
            ~PhysicsTaskScheduler();

            /// @brief move actors taking into account desired movements and collisions
            /// @param numSteps how much simulation step to run
            /// @param timeAccum accumulated time from previous run to interpolate movements
            /// @param actorsData per actor data needed to compute new positions
            /// @return new position of each actor
            const PtrPositionList& moveActors(int numSteps, float timeAccum, std::vector<ActorFrameData>&& actorsData, CollisionMap& standingCollisions);

            // Thread safe wrappers
            void rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld, btCollisionWorld::RayResultCallback& resultCallback) const; 
            void convexSweepTest(const btConvexShape* castShape, const btTransform& from, const btTransform& to, btCollisionWorld::ConvexResultCallback& resultCallback) const;
            void contactTest(btCollisionObject* colObj, btCollisionWorld::ContactResultCallback& resultCallback);
            boost::optional<btVector3> getHitPoint(const btTransform& from, btCollisionObject* target);
            void aabbTest(const btVector3& aabbMin, const btVector3& aabbMax, btBroadphaseAabbCallback& callback);
            void getAabb(const btCollisionObject* obj, btVector3& min, btVector3& max);
            void setCollisionFilterMask(btCollisionObject* collisionObject, int collisionFilterMask);
            void addCollisionObject(btCollisionObject* collisionObject, int collisionFilterGroup, int collisionFilterMask);
            void removeCollisionObject(btCollisionObject* collisionObject);
            void updateSingleAabb(std::weak_ptr<PtrHolder> ptr);
            bool getLineOfSight(const std::weak_ptr<Actor>& actor1, const std::weak_ptr<Actor>& actor2);

        private:
            /// @brief Synchronize several threads
            class Barrier
            {
                public:
                    /// @param count number of threads to wait on
                    explicit Barrier(int count);
                    /// @brief stop execution of threads until count distinct threads reach this point
                    /// @param func callable to be executed once after all threads have met
                    template<typename F>
                    void wait(F&& func);

                private:
                    int mThreadCount;
                    int mRendezvousCount;
                    int mGeneration;
                    mutable std::mutex mMutex;
                    std::condition_variable mRendezvous;

            };

            void syncComputation();
            void asyncComputation();
            void worker();
            void perSimulationStepUpdate();
            void perFrameUpdate();
            bool hasLineOfSight(const Actor* actor1, const Actor* actor2);
            void refreshLOSCache();
            void updateAabbs();

            std::vector<ActorFrameData> mActorsFrameData;
            PtrPositionList mMovementResults;
            PtrPositionList mPreviousMovementResults;
            const float mPhysicsDt;
            float mTimeAccum;
            std::shared_ptr<btCollisionWorld> mCollisionWorld;
            CollisionMap mStandingCollisions;
            std::vector<LOSRequest> mLOSCache;
            std::set<std::weak_ptr<PtrHolder>, std::owner_less<std::weak_ptr<PtrHolder>>> mUpdateAabb;
            std::unique_ptr<Barrier> mBarrier; // TODO: use std::experimental::flex_barrier or std::barrier once it becomes a thing

            int mNumThreads;
            int mNumJobs;
            int mRemainingSteps;
            int mLOSCacheExpiry;
            bool mDeferAabbUpdate;
            bool mNewFrame;
            bool mAdvanceSimulation;
            bool mThreadSafeBullet;
            std::atomic<int> mNextJob;
            std::atomic<int> mNextLOS;
            bool mQuit;
            std::vector<std::thread> mThreads;

            mutable std::shared_timed_mutex mSimulationMutex;
            mutable std::shared_timed_mutex mCollisionWorldMutex;
            mutable std::shared_timed_mutex mLOSCacheMutex;
            mutable std::mutex mUpdateAabbMutex;
            std::condition_variable_any mHasJob;
    };

}
#endif
