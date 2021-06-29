#ifndef OPENMW_NAVMESHTOOL_WORLDSPACEDATA_H
#define OPENMW_NAVMESHTOOL_WORLDSPACEDATA_H

#include <components/bullethelpers/collisionobject.hpp>
#include <components/detournavigator/tilecachedrecastmeshmanager.hpp>
#include <components/esm/loadland.hpp>
#include <components/misc/convert.hpp>
#include <components/resource/bulletshape.hpp>

#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/Gimpact/btBoxCollision.h>
#include <LinearMath/btVector3.h>

#include <memory>
#include <string>
#include <vector>

namespace ESM
{
    class ESMReader;
}

namespace VFS
{
    class Manager;
}

namespace Resource
{
    class BulletShapeManager;
}

namespace DetourNavigator
{
    struct Settings;
}

namespace NavMeshTool
{
    struct Content;

    using DetourNavigator::TileCachedRecastMeshManager;

    struct WorldspaceNavMeshInput
    {
        std::string mWorldspace;
        TileCachedRecastMeshManager mTileCachedRecastMeshManager;
        btAABB mAabb;
        bool mAabbInitialized = false;

        explicit WorldspaceNavMeshInput(std::string worldspace, const DetourNavigator::Settings& settings);
    };

    class BulletObject
    {
    public:
        BulletObject(osg::ref_ptr<Resource::BulletShapeInstance>&& shapeInstance, const float (&position)[3],
                btScalar localScaling, const btQuaternion& rotation)
            : mShapeInstance(std::move(shapeInstance))
            , mCollisionObject(BulletHelpers::makeCollisionObject(mShapeInstance->getCollisionShape(), position, rotation))
        {
            mShapeInstance->setLocalScaling(btVector3(localScaling, localScaling, localScaling));
        }

        BulletObject(osg::ref_ptr<Resource::BulletShapeInstance>&& shapeInstance, const ESM::Position& position,
                btScalar localScaling)
            : BulletObject(std::move(shapeInstance), position.pos, localScaling,
                            Misc::Convert::toBullet(Misc::Convert::makeOsgQuat(position)))
        {}

        btCollisionObject& getCollisionObject() const noexcept { return *mCollisionObject; }
        const osg::ref_ptr<Resource::BulletShapeInstance>& getShapeInstance() const noexcept { return mShapeInstance; }

    private:
        osg::ref_ptr<Resource::BulletShapeInstance> mShapeInstance;
        std::unique_ptr<btCollisionObject> mCollisionObject;
    };

    struct WorldspaceData
    {
        std::vector<std::unique_ptr<WorldspaceNavMeshInput>> mNavMeshInputs;
        std::vector<BulletObject> mObjects;
        std::vector<std::unique_ptr<ESM::Land::LandData>> mLandData;
        std::vector<std::vector<float>> mHeightfields;
    };

    WorldspaceData gatherWorldspaceData(const DetourNavigator::Settings& settings, std::vector<ESM::ESMReader>& readers,
        const VFS::Manager& vfs, Resource::BulletShapeManager& bulletShapeManager, const Content& content,
        bool processInteriorCells);
}

#endif
