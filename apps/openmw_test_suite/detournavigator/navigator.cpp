#include "operators.hpp"
#include "settings.hpp"

#include <components/bullethelpers/heightfield.hpp>
#include <components/detournavigator/navigatorimpl.hpp>
#include <components/detournavigator/navigatorutils.hpp>
#include <components/detournavigator/navmeshdb.hpp>
#include <components/esm3/loadland.hpp>
#include <components/loadinglistener/loadinglistener.hpp>
#include <components/misc/rng.hpp>
#include <components/resource/bulletshape.hpp>

#include <osg/io_utils>
#include <osg/ref_ptr>

#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <array>
#include <condition_variable>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

MATCHER_P3(Vec3fEq, x, y, z, "")
{
    return std::abs(arg.x() - x) < 1e-3 && std::abs(arg.y() - y) < 1e-3 && std::abs(arg.z() - z) < 1e-3;
}

namespace
{
    using namespace testing;
    using namespace DetourNavigator;
    using namespace DetourNavigator::Tests;

    struct DetourNavigatorNavigatorTest : Test
    {
        Settings mSettings = makeSettings();
        std::unique_ptr<Navigator> mNavigator;
        const osg::Vec3f mPlayerPosition;
        const std::string mWorldspace;
        const AgentBounds mAgentBounds{ CollisionShapeType::Aabb, { 29, 29, 66 } };
        osg::Vec3f mStart;
        osg::Vec3f mEnd;
        std::deque<osg::Vec3f> mPath;
        std::back_insert_iterator<std::deque<osg::Vec3f>> mOut;
        AreaCosts mAreaCosts;
        Loading::Listener mListener;
        const osg::Vec2i mCellPosition{ 0, 0 };
        const int mHeightfieldTileSize = ESM::Land::REAL_SIZE / (ESM::Land::LAND_SIZE - 1);
        const float mEndTolerance = 0;
        const btTransform mTransform{ btMatrix3x3::getIdentity(), btVector3(256, 256, 0) };
        const ObjectTransform mObjectTransform{ ESM::Position{ { 256, 256, 0 }, { 0, 0, 0 } }, 0.0f };

        DetourNavigatorNavigatorTest()
            : mPlayerPosition(256, 256, 0)
            , mWorldspace("sys::default")
            , mStart(52, 460, 1)
            , mEnd(460, 52, 1)
            , mOut(mPath)
        {
            mNavigator.reset(new NavigatorImpl(
                mSettings, std::make_unique<NavMeshDb>(":memory:", std::numeric_limits<std::uint64_t>::max())));
        }
    };

    template <std::size_t size>
    std::unique_ptr<btHeightfieldTerrainShape> makeSquareHeightfieldTerrainShape(
        const std::array<btScalar, size>& values, btScalar heightScale = 1, int upAxis = 2,
        PHY_ScalarType heightDataType = PHY_FLOAT, bool flipQuadEdges = false)
    {
        const int width = static_cast<int>(std::sqrt(size));
        const btScalar min = *std::min_element(values.begin(), values.end());
        const btScalar max = *std::max_element(values.begin(), values.end());
        const btScalar greater = std::max(std::abs(min), std::abs(max));
        return std::make_unique<btHeightfieldTerrainShape>(
            width, width, values.data(), heightScale, -greater, greater, upAxis, heightDataType, flipQuadEdges);
    }

    template <std::size_t size>
    HeightfieldSurface makeSquareHeightfieldSurface(const std::array<float, size>& values)
    {
        const auto [min, max] = std::minmax_element(values.begin(), values.end());
        const float greater = std::max(std::abs(*min), std::abs(*max));
        HeightfieldSurface surface;
        surface.mHeights = values.data();
        surface.mMinHeight = -greater;
        surface.mMaxHeight = greater;
        surface.mSize = static_cast<int>(std::sqrt(size));
        return surface;
    }

    template <class T>
    osg::ref_ptr<const Resource::BulletShapeInstance> makeBulletShapeInstance(std::unique_ptr<T>&& shape)
    {
        osg::ref_ptr<Resource::BulletShape> bulletShape(new Resource::BulletShape);
        bulletShape->mCollisionShape.reset(std::move(shape).release());
        return new Resource::BulletShapeInstance(bulletShape);
    }

    template <class T>
    class CollisionShapeInstance
    {
    public:
        CollisionShapeInstance(std::unique_ptr<T>&& shape)
            : mInstance(makeBulletShapeInstance(std::move(shape)))
        {
        }

        T& shape() { return static_cast<T&>(*mInstance->mCollisionShape); }
        const osg::ref_ptr<const Resource::BulletShapeInstance>& instance() const { return mInstance; }

    private:
        osg::ref_ptr<const Resource::BulletShapeInstance> mInstance;
    };

    btVector3 getHeightfieldShift(const osg::Vec2i& cellPosition, int cellSize, float minHeight, float maxHeight)
    {
        return BulletHelpers::getHeightfieldShift(cellPosition.x(), cellPosition.x(), cellSize, minHeight, maxHeight);
    }

    TEST_F(DetourNavigatorNavigatorTest, find_path_for_empty_should_return_empty)
    {
        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, mOut),
            Status::NavMeshNotFound);
        EXPECT_EQ(mPath, std::deque<osg::Vec3f>());
    }

    TEST_F(DetourNavigatorNavigatorTest, find_path_for_existing_agent_with_no_navmesh_should_throw_exception)
    {
        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, mOut),
            Status::StartPolygonNotFound);
    }

    TEST_F(DetourNavigatorNavigatorTest, add_agent_should_count_each_agent)
    {
        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->removeAgent(mAgentBounds);
        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, mOut),
            Status::StartPolygonNotFound);
    }

    TEST_F(DetourNavigatorNavigatorTest, update_then_find_path_should_return_path)
    {
        constexpr std::array<float, 5 * 5> heightfieldData{ {
            0, 0, 0, 0, 0, // row 0
            0, -25, -25, -25, -25, // row 1
            0, -25, -100, -100, -100, // row 2
            0, -25, -100, -100, -100, // row 3
            0, -25, -100, -100, -100, // row 4
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = mHeightfieldTileSize * (surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        auto updateGuard = mNavigator->makeUpdateGuard();
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, updateGuard.get());
        mNavigator->update(mPlayerPosition, updateGuard.get());
        updateGuard.reset();
        mNavigator->wait(WaitConditionType::requiredTilesPresent, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 1.99999392032623291015625),
                Vec3fEq(460, 56.66664886474609375, 1.99999392032623291015625)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, find_path_to_the_start_position_should_contain_single_point)
    {
        constexpr std::array<float, 5 * 5> heightfieldData{ {
            0, 0, 0, 0, 0, // row 0
            0, -25, -25, -25, -25, // row 1
            0, -25, -100, -100, -100, // row 2
            0, -25, -100, -100, -100, // row 3
            0, -25, -100, -100, -100, // row 4
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = mHeightfieldTileSize * (surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        auto updateGuard = mNavigator->makeUpdateGuard();
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, updateGuard.get());
        mNavigator->update(mPlayerPosition, updateGuard.get());
        updateGuard.reset();
        mNavigator->wait(WaitConditionType::requiredTilesPresent, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mStart, Flag_walk, mAreaCosts, mEndTolerance, mOut),
            Status::Success);

        EXPECT_THAT(mPath, ElementsAre(Vec3fEq(56.66666412353515625, 460, 1.99998295307159423828125))) << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, add_object_should_change_navmesh)
    {
        mSettings.mWaitUntilMinDistanceToPlayer = 0;
        mNavigator.reset(new NavigatorImpl(
            mSettings, std::make_unique<NavMeshDb>(":memory:", std::numeric_limits<std::uint64_t>::max())));

        const std::array<float, 5 * 5> heightfieldData{ {
            0, 0, 0, 0, 0, // row 0
            0, -25, -25, -25, -25, // row 1
            0, -25, -100, -100, -100, // row 2
            0, -25, -100, -100, -100, // row 3
            0, -25, -100, -100, -100, // row 4
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = mHeightfieldTileSize * (surface.mSize - 1);

        CollisionShapeInstance compound(std::make_unique<btCompoundShape>());
        compound.shape().addChildShape(
            btTransform(btMatrix3x3::getIdentity(), btVector3(0, 0, 0)), new btBoxShape(btVector3(20, 20, 100)));

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66666412353515625, 460, 1.99998295307159423828125),
                Vec3fEq(460, 56.66666412353515625, 1.99998295307159423828125)))
            << mPath;

        {
            auto updateGuard = mNavigator->makeUpdateGuard();
            mNavigator->addObject(ObjectId(&compound.shape()), ObjectShapes(compound.instance(), mObjectTransform),
                mTransform, updateGuard.get());
            mNavigator->update(mPlayerPosition, updateGuard.get());
        }
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mPath.clear();
        mOut = std::back_inserter(mPath);
        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 1.99999392032623291015625),
                Vec3fEq(181.33331298828125, 215.33331298828125, -20.6666717529296875),
                Vec3fEq(215.33331298828125, 181.33331298828125, -20.6666717529296875),
                Vec3fEq(460, 56.66664886474609375, 1.99999392032623291015625)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, update_changed_object_should_change_navmesh)
    {
        const std::array<float, 5 * 5> heightfieldData{ {
            0, 0, 0, 0, 0, // row 0
            0, -25, -25, -25, -25, // row 1
            0, -25, -100, -100, -100, // row 2
            0, -25, -100, -100, -100, // row 3
            0, -25, -100, -100, -100, // row 4
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = mHeightfieldTileSize * (surface.mSize - 1);

        CollisionShapeInstance compound(std::make_unique<btCompoundShape>());
        compound.shape().addChildShape(
            btTransform(btMatrix3x3::getIdentity(), btVector3(0, 0, 0)), new btBoxShape(btVector3(20, 20, 100)));

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->addObject(
            ObjectId(&compound.shape()), ObjectShapes(compound.instance(), mObjectTransform), mTransform, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 1.99999392032623291015625),
                Vec3fEq(181.33331298828125, 215.33331298828125, -20.6666717529296875),
                Vec3fEq(215.33331298828125, 181.33331298828125, -20.6666717529296875),
                Vec3fEq(460, 56.66664886474609375, 1.99999392032623291015625)))
            << mPath;

        compound.shape().updateChildTransform(0, btTransform(btMatrix3x3::getIdentity(), btVector3(1000, 0, 0)));

        mNavigator->updateObject(
            ObjectId(&compound.shape()), ObjectShapes(compound.instance(), mObjectTransform), mTransform, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mPath.clear();
        mOut = std::back_inserter(mPath);
        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 1.99999392032623291015625),
                Vec3fEq(460, 56.66664886474609375, 1.99999392032623291015625)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, for_overlapping_heightfields_objects_should_use_higher)
    {
        const std::array<btScalar, 5 * 5> heightfieldData1{ {
            0, 0, 0, 0, 0, // row 0
            0, -25, -25, -25, -25, // row 1
            0, -25, -100, -100, -100, // row 2
            0, -25, -100, -100, -100, // row 3
            0, -25, -100, -100, -100, // row 4
        } };
        CollisionShapeInstance heightfield1(makeSquareHeightfieldTerrainShape(heightfieldData1));
        heightfield1.shape().setLocalScaling(btVector3(128, 128, 1));

        const std::array<btScalar, 5 * 5> heightfieldData2{ {
            -25, -25, -25, -25, -25, // row 0
            -25, -25, -25, -25, -25, // row 1
            -25, -25, -25, -25, -25, // row 2
            -25, -25, -25, -25, -25, // row 3
            -25, -25, -25, -25, -25, // row 4
        } };
        CollisionShapeInstance heightfield2(makeSquareHeightfieldTerrainShape(heightfieldData2));
        heightfield2.shape().setLocalScaling(btVector3(128, 128, 1));

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addObject(ObjectId(&heightfield1.shape()), ObjectShapes(heightfield1.instance(), mObjectTransform),
            mTransform, nullptr);
        mNavigator->addObject(ObjectId(&heightfield2.shape()), ObjectShapes(heightfield2.instance(), mObjectTransform),
            mTransform, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 1.99999392032623291015625),
                Vec3fEq(460, 56.66664886474609375, 1.99999392032623291015625)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, only_one_heightfield_per_cell_is_allowed)
    {
        const std::array<float, 5 * 5> heightfieldData1{ {
            0, 0, 0, 0, 0, // row 0
            0, -25, -25, -25, -25, // row 1
            0, -25, -100, -100, -100, // row 2
            0, -25, -100, -100, -100, // row 3
            0, -25, -100, -100, -100, // row 4
        } };
        const HeightfieldSurface surface1 = makeSquareHeightfieldSurface(heightfieldData1);
        const int cellSize1 = mHeightfieldTileSize * (surface1.mSize - 1);

        const std::array<float, 5 * 5> heightfieldData2{ {
            -25, -25, -25, -25, -25, // row 0
            -25, -25, -25, -25, -25, // row 1
            -25, -25, -25, -25, -25, // row 2
            -25, -25, -25, -25, -25, // row 3
            -25, -25, -25, -25, -25, // row 4
        } };
        const HeightfieldSurface surface2 = makeSquareHeightfieldSurface(heightfieldData2);
        const int cellSize2 = mHeightfieldTileSize * (surface2.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize1, surface1, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const Version version = mNavigator->getNavMesh(mAgentBounds)->lockConst()->getVersion();

        mNavigator->addHeightfield(mCellPosition, cellSize2, surface2, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(mNavigator->getNavMesh(mAgentBounds)->lockConst()->getVersion(), version);
    }

    TEST_F(DetourNavigatorNavigatorTest, path_should_be_around_avoid_shape)
    {
        osg::ref_ptr<Resource::BulletShape> bulletShape(new Resource::BulletShape);

        std::array<btScalar, 5 * 5> heightfieldData{ {
            0, 0, 0, 0, 0, // row 0
            0, -25, -25, -25, -25, // row 1
            0, -25, -100, -100, -100, // row 2
            0, -25, -100, -100, -100, // row 3
            0, -25, -100, -100, -100, // row 4
        } };
        std::unique_ptr<btHeightfieldTerrainShape> shapePtr = makeSquareHeightfieldTerrainShape(heightfieldData);
        shapePtr->setLocalScaling(btVector3(128, 128, 1));
        bulletShape->mCollisionShape.reset(shapePtr.release());

        std::array<btScalar, 5 * 5> heightfieldDataAvoid{ {
            -25, -25, -25, -25, -25, // row 0
            -25, -25, -25, -25, -25, // row 1
            -25, -25, -25, -25, -25, // row 2
            -25, -25, -25, -25, -25, // row 3
            -25, -25, -25, -25, -25, // row 4
        } };
        std::unique_ptr<btHeightfieldTerrainShape> shapeAvoidPtr
            = makeSquareHeightfieldTerrainShape(heightfieldDataAvoid);
        shapeAvoidPtr->setLocalScaling(btVector3(128, 128, 1));
        bulletShape->mAvoidCollisionShape.reset(shapeAvoidPtr.release());

        osg::ref_ptr<const Resource::BulletShapeInstance> instance(new Resource::BulletShapeInstance(bulletShape));

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addObject(
            ObjectId(instance->mCollisionShape.get()), ObjectShapes(instance, mObjectTransform), mTransform, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 1.99999392032623291015625),
                Vec3fEq(158.6666412353515625, 249.3332977294921875, -20.6666717529296875),
                Vec3fEq(249.3332977294921875, 158.6666412353515625, -20.6666717529296875),
                Vec3fEq(460, 56.66664886474609375, 1.99999392032623291015625)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, path_should_be_over_water_ground_lower_than_water_with_only_swim_flag)
    {
        std::array<float, 5 * 5> heightfieldData{ {
            -50, -50, -50, -50, 0, // row 0
            -50, -100, -150, -100, -50, // row 1
            -50, -150, -200, -150, -100, // row 2
            -50, -100, -150, -100, -100, // row 3
            0, -50, -100, -100, -100, // row 4
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = mHeightfieldTileSize * (surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addWater(mCellPosition, cellSize, 300, nullptr);
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mStart.x() = 256;
        mStart.z() = 300;
        mEnd.x() = 256;
        mEnd.z() = 300;

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_swim, mAreaCosts, mEndTolerance, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(256, 460, 185.3333282470703125), Vec3fEq(256, 56.66664886474609375, 185.3333282470703125)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, path_should_be_over_water_when_ground_cross_water_with_swim_and_walk_flags)
    {
        std::array<float, 7 * 7> heightfieldData{ {
            0, 0, 0, 0, 0, 0, 0, // row 0
            0, -100, -100, -100, -100, -100, 0, // row 1
            0, -100, -150, -150, -150, -100, 0, // row 2
            0, -100, -150, -200, -150, -100, 0, // row 3
            0, -100, -150, -150, -150, -100, 0, // row 4
            0, -100, -100, -100, -100, -100, 0, // row 5
            0, 0, 0, 0, 0, 0, 0, // row 6
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = mHeightfieldTileSize * (surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addWater(mCellPosition, cellSize, -25, nullptr);
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mStart.x() = 256;
        mEnd.x() = 256;

        EXPECT_EQ(
            findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_swim | Flag_walk, mAreaCosts, mEndTolerance, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(256, 460, -129.4098663330078125), Vec3fEq(256, 56.66664886474609375, -30.0000133514404296875)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest,
        path_should_be_over_water_when_ground_cross_water_with_max_int_cells_size_and_swim_and_walk_flags)
    {
        std::array<float, 7 * 7> heightfieldData{ {
            0, 0, 0, 0, 0, 0, 0, // row 0
            0, -100, -100, -100, -100, -100, 0, // row 1
            0, -100, -150, -150, -150, -100, 0, // row 2
            0, -100, -150, -200, -150, -100, 0, // row 3
            0, -100, -150, -150, -150, -100, 0, // row 4
            0, -100, -100, -100, -100, -100, 0, // row 5
            0, 0, 0, 0, 0, 0, 0, // row 6
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = mHeightfieldTileSize * (surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->addWater(mCellPosition, std::numeric_limits<int>::max(), -25, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mStart.x() = 256;
        mEnd.x() = 256;

        EXPECT_EQ(
            findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_swim | Flag_walk, mAreaCosts, mEndTolerance, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre(
                Vec3fEq(256, 460, -129.4098663330078125), Vec3fEq(256, 56.66664886474609375, -30.0000133514404296875)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, path_should_be_over_ground_when_ground_cross_water_with_only_walk_flag)
    {
        std::array<float, 7 * 7> heightfieldData{ {
            0, 0, 0, 0, 0, 0, 0, // row 0
            0, -100, -100, -100, -100, -100, 0, // row 1
            0, -100, -150, -150, -150, -100, 0, // row 2
            0, -100, -150, -200, -150, -100, 0, // row 3
            0, -100, -150, -150, -150, -100, 0, // row 4
            0, -100, -100, -100, -100, -100, 0, // row 5
            0, 0, 0, 0, 0, 0, 0, // row 6
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = mHeightfieldTileSize * (surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addWater(mCellPosition, cellSize, -25, nullptr);
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mStart.x() = 256;
        mEnd.x() = 256;

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(256, 460, -129.4098663330078125), Vec3fEq(256, 56.66664886474609375, -30.0000133514404296875)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, update_object_remove_and_update_then_find_path_should_return_path)
    {
        const std::array<btScalar, 5 * 5> heightfieldData{ {
            0, 0, 0, 0, 0, // row 0
            0, -25, -25, -25, -25, // row 1
            0, -25, -100, -100, -100, // row 2
            0, -25, -100, -100, -100, // row 3
            0, -25, -100, -100, -100, // row 4
        } };
        CollisionShapeInstance heightfield(makeSquareHeightfieldTerrainShape(heightfieldData));
        heightfield.shape().setLocalScaling(btVector3(128, 128, 1));

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addObject(ObjectId(&heightfield.shape()), ObjectShapes(heightfield.instance(), mObjectTransform),
            mTransform, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mNavigator->removeObject(ObjectId(&heightfield.shape()), nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mNavigator->addObject(ObjectId(&heightfield.shape()), ObjectShapes(heightfield.instance(), mObjectTransform),
            mTransform, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 1.99999392032623291015625),
                Vec3fEq(460, 56.66664886474609375, 1.99999392032623291015625)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, update_heightfield_remove_and_update_then_find_path_should_return_path)
    {
        const std::array<float, 5 * 5> heightfieldData{ {
            0, 0, 0, 0, 0, // row 0
            0, -25, -25, -25, -25, // row 1
            0, -25, -100, -100, -100, // row 2
            0, -25, -100, -100, -100, // row 3
            0, -25, -100, -100, -100, // row 4
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = mHeightfieldTileSize * (surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mNavigator->removeHeightfield(mCellPosition, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 1.99999392032623291015625),
                Vec3fEq(460, 56.66664886474609375, 1.99999392032623291015625)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, update_then_find_random_point_around_circle_should_return_position)
    {
        const std::array<float, 6 * 6> heightfieldData{ {
            0, 0, 0, 0, 0, 0, // row 0
            0, -25, -25, -25, -25, -25, // row 1
            0, -25, -1000, -1000, -100, -100, // row 2
            0, -25, -1000, -1000, -100, -100, // row 3
            0, -25, -100, -100, -100, -100, // row 4
            0, -25, -100, -100, -100, -100, // row 5
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = mHeightfieldTileSize * (surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        Misc::Rng::init(42);

        const auto result = findRandomPointAroundCircle(
            *mNavigator, mAgentBounds, mStart, 100.0, Flag_walk, []() { return Misc::Rng::rollClosedProbability(); });

        ASSERT_THAT(result, Optional(Vec3fEq(70.35845947265625, 335.592041015625, -2.6667339801788330078125)))
            << (result ? *result : osg::Vec3f());

        const auto distance = (*result - mStart).length();

        EXPECT_FLOAT_EQ(distance, 125.80865478515625) << distance;
    }

    TEST_F(DetourNavigatorNavigatorTest, multiple_threads_should_lock_tiles)
    {
        mSettings.mAsyncNavMeshUpdaterThreads = 2;
        mNavigator.reset(new NavigatorImpl(
            mSettings, std::make_unique<NavMeshDb>(":memory:", std::numeric_limits<std::uint64_t>::max())));

        const std::array<float, 5 * 5> heightfieldData{ {
            0, 0, 0, 0, 0, // row 0
            0, -25, -25, -25, -25, // row 1
            0, -25, -100, -100, -100, // row 2
            0, -25, -100, -100, -100, // row 3
            0, -25, -100, -100, -100, // row 4
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = mHeightfieldTileSize * (surface.mSize - 1);
        const btVector3 shift = getHeightfieldShift(mCellPosition, cellSize, surface.mMinHeight, surface.mMaxHeight);

        std::vector<CollisionShapeInstance<btBoxShape>> boxes;
        std::generate_n(
            std::back_inserter(boxes), 100, [] { return std::make_unique<btBoxShape>(btVector3(20, 20, 100)); });

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));

        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);

        for (std::size_t i = 0; i < boxes.size(); ++i)
        {
            const btTransform transform(
                btMatrix3x3::getIdentity(), btVector3(shift.x() + i * 10, shift.y() + i * 10, i * 10));
            mNavigator->addObject(
                ObjectId(&boxes[i].shape()), ObjectShapes(boxes[i].instance(), mObjectTransform), transform, nullptr);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(1));

        for (std::size_t i = 0; i < boxes.size(); ++i)
        {
            const btTransform transform(
                btMatrix3x3::getIdentity(), btVector3(shift.x() + i * 10 + 1, shift.y() + i * 10 + 1, i * 10 + 1));
            mNavigator->updateObject(
                ObjectId(&boxes[i].shape()), ObjectShapes(boxes[i].instance(), mObjectTransform), transform, nullptr);
        }

        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 1.99999392032623291015625),
                Vec3fEq(181.33331298828125, 215.33331298828125, -20.6666717529296875),
                Vec3fEq(215.33331298828125, 181.33331298828125, -20.6666717529296875),
                Vec3fEq(460, 56.66664886474609375, 1.99999392032623291015625)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, update_changed_multiple_times_object_should_delay_navmesh_change)
    {
        std::vector<CollisionShapeInstance<btBoxShape>> shapes;
        std::generate_n(
            std::back_inserter(shapes), 100, [] { return std::make_unique<btBoxShape>(btVector3(64, 64, 64)); });

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));

        for (std::size_t i = 0; i < shapes.size(); ++i)
        {
            const btTransform transform(btMatrix3x3::getIdentity(), btVector3(i * 32, i * 32, i * 32));
            mNavigator->addObject(
                ObjectId(&shapes[i].shape()), ObjectShapes(shapes[i].instance(), mObjectTransform), transform, nullptr);
        }
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const auto start = std::chrono::steady_clock::now();
        for (std::size_t i = 0; i < shapes.size(); ++i)
        {
            const btTransform transform(btMatrix3x3::getIdentity(), btVector3(i * 32 + 1, i * 32 + 1, i * 32 + 1));
            mNavigator->updateObject(
                ObjectId(&shapes[i].shape()), ObjectShapes(shapes[i].instance(), mObjectTransform), transform, nullptr);
        }
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        for (std::size_t i = 0; i < shapes.size(); ++i)
        {
            const btTransform transform(btMatrix3x3::getIdentity(), btVector3(i * 32 + 2, i * 32 + 2, i * 32 + 2));
            mNavigator->updateObject(
                ObjectId(&shapes[i].shape()), ObjectShapes(shapes[i].instance(), mObjectTransform), transform, nullptr);
        }
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const auto duration = std::chrono::steady_clock::now() - start;

        EXPECT_GT(duration, mSettings.mMinUpdateInterval)
            << std::chrono::duration_cast<std::chrono::duration<float, std::milli>>(duration).count() << " ms";
    }

    TEST_F(DetourNavigatorNavigatorTest, update_then_raycast_should_return_position)
    {
        const std::array<float, 5 * 5> heightfieldData{ {
            0, 0, 0, 0, 0, // row 0
            0, -25, -25, -25, -25, // row 1
            0, -25, -100, -100, -100, // row 2
            0, -25, -100, -100, -100, // row 3
            0, -25, -100, -100, -100, // row 4
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = mHeightfieldTileSize * (surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const osg::Vec3f start(57, 460, 1);
        const osg::Vec3f end(460, 57, 1);
        const auto result = raycast(*mNavigator, mAgentBounds, start, end, Flag_walk);

        ASSERT_THAT(result, Optional(Vec3fEq(end.x(), end.y(), 1.95257937908172607421875)))
            << (result ? *result : osg::Vec3f());
    }

    TEST_F(DetourNavigatorNavigatorTest,
        update_for_oscillating_object_that_does_not_change_navmesh_should_not_trigger_navmesh_update)
    {
        const std::array<float, 5 * 5> heightfieldData{ {
            0, 0, 0, 0, 0, // row 0
            0, -25, -25, -25, -25, // row 1
            0, -25, -100, -100, -100, // row 2
            0, -25, -100, -100, -100, // row 3
            0, -25, -100, -100, -100, // row 4
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = mHeightfieldTileSize * (surface.mSize - 1);

        CollisionShapeInstance oscillatingBox(std::make_unique<btBoxShape>(btVector3(20, 20, 20)));
        const btVector3 oscillatingBoxShapePosition(288, 288, 400);
        CollisionShapeInstance borderBox(std::make_unique<btBoxShape>(btVector3(50, 50, 50)));

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->addObject(ObjectId(&oscillatingBox.shape()),
            ObjectShapes(oscillatingBox.instance(), mObjectTransform),
            btTransform(btMatrix3x3::getIdentity(), oscillatingBoxShapePosition), nullptr);
        // add this box to make navmesh bound box independent from oscillatingBoxShape rotations
        mNavigator->addObject(ObjectId(&borderBox.shape()), ObjectShapes(borderBox.instance(), mObjectTransform),
            btTransform(btMatrix3x3::getIdentity(), oscillatingBoxShapePosition + btVector3(0, 0, 200)), nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const Version expectedVersion{ 1, 4 };

        const auto navMeshes = mNavigator->getNavMeshes();
        ASSERT_EQ(navMeshes.size(), 1);
        ASSERT_EQ(navMeshes.begin()->second->lockConst()->getVersion(), expectedVersion);

        for (int n = 0; n < 10; ++n)
        {
            const btTransform transform(
                btQuaternion(btVector3(0, 0, 1), n * 2 * osg::PI / 10), oscillatingBoxShapePosition);
            mNavigator->updateObject(ObjectId(&oscillatingBox.shape()),
                ObjectShapes(oscillatingBox.instance(), mObjectTransform), transform, nullptr);
            mNavigator->update(mPlayerPosition, nullptr);
            mNavigator->wait(WaitConditionType::allJobsDone, &mListener);
        }

        ASSERT_EQ(navMeshes.size(), 1);
        ASSERT_EQ(navMeshes.begin()->second->lockConst()->getVersion(), expectedVersion);
    }

    TEST_F(DetourNavigatorNavigatorTest, should_provide_path_over_flat_heightfield)
    {
        const HeightfieldPlane plane{ 100 };
        const int cellSize = mHeightfieldTileSize * 4;

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, plane, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::requiredTilesPresent, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 102), Vec3fEq(460, 56.66664886474609375, 102)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, for_not_reachable_destination_find_path_should_provide_partial_path)
    {
        const std::array<float, 5 * 5> heightfieldData{ {
            0, 0, 0, 0, 0, // row 0
            0, -25, -25, -25, -25, // row 1
            0, -25, -100, -100, -100, // row 2
            0, -25, -100, -100, -100, // row 3
            0, -25, -100, -100, -100, // row 4
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = mHeightfieldTileSize * (surface.mSize - 1);

        CollisionShapeInstance compound(std::make_unique<btCompoundShape>());
        compound.shape().addChildShape(btTransform(btMatrix3x3::getIdentity(), btVector3(204, -204, 0)),
            new btBoxShape(btVector3(200, 200, 1000)));

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->addObject(
            ObjectId(&compound.shape()), ObjectShapes(compound.instance(), mObjectTransform), mTransform, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, mOut),
            Status::PartialPath);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, -2.5371119976043701171875),
                Vec3fEq(222, 290, -71.33342742919921875)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, end_tolerance_should_extent_available_destinations)
    {
        const std::array<float, 5 * 5> heightfieldData{ {
            0, 0, 0, 0, 0, // row 0
            0, -25, -25, -25, -25, // row 1
            0, -25, -100, -100, -100, // row 2
            0, -25, -100, -100, -100, // row 3
            0, -25, -100, -100, -100, // row 4
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = mHeightfieldTileSize * (surface.mSize - 1);

        CollisionShapeInstance compound(std::make_unique<btCompoundShape>());
        compound.shape().addChildShape(btTransform(btMatrix3x3::getIdentity(), btVector3(204, -204, 0)),
            new btBoxShape(btVector3(100, 100, 1000)));

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->addObject(
            ObjectId(&compound.shape()), ObjectShapes(compound.instance(), mObjectTransform), mTransform, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const float endTolerance = 1000.0f;

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, endTolerance, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, -2.5371119976043701171875),
                Vec3fEq(305.999969482421875, 56.66664886474609375, -2.6667406558990478515625)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, only_one_water_per_cell_is_allowed)
    {
        const int cellSize1 = 100;
        const float level1 = 1;
        const int cellSize2 = 200;
        const float level2 = 2;

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addWater(mCellPosition, cellSize1, level1, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const Version version = mNavigator->getNavMesh(mAgentBounds)->lockConst()->getVersion();

        mNavigator->addWater(mCellPosition, cellSize2, level2, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(mNavigator->getNavMesh(mAgentBounds)->lockConst()->getVersion(), version);
    }

    TEST_F(DetourNavigatorNavigatorTest, update_for_very_big_object_should_be_limited)
    {
        const float size = static_cast<float>(2 * static_cast<std::int64_t>(std::numeric_limits<int>::max()) - 1);
        CollisionShapeInstance bigBox(std::make_unique<btBoxShape>(btVector3(size, size, 1)));
        const ObjectTransform objectTransform{
            .mPosition = ESM::Position{ .pos = { 0, 0, 0 }, .rot{ 0, 0, 0 } },
            .mScale = 1.0f,
        };

        mNavigator->updateBounds(mPlayerPosition, nullptr);
        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addObject(ObjectId(&bigBox.shape()), ObjectShapes(bigBox.instance(), objectTransform),
            btTransform::getIdentity(), nullptr);

        bool updated = false;
        std::condition_variable updateFinished;
        std::mutex mutex;

        std::thread thread([&] {
            mNavigator->update(mPlayerPosition, nullptr);
            std::lock_guard lock(mutex);
            updated = true;
            updateFinished.notify_all();
        });

        {
            std::unique_lock lock(mutex);
            updateFinished.wait_for(lock, std::chrono::seconds(3), [&] { return updated; });
            ASSERT_TRUE(updated);
        }

        thread.join();

        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(mNavigator->getRecastMeshTiles().size(), 509);

        const auto navMesh = mNavigator->getNavMesh(mAgentBounds);
        ASSERT_NE(navMesh, nullptr);

        std::size_t usedNavMeshTiles = 0;
        navMesh->lockConst()->forEachUsedTile([&](const auto&...) { ++usedNavMeshTiles; });
        EXPECT_EQ(usedNavMeshTiles, 509);
    }

    struct DetourNavigatorNavigatorNotSupportedAgentBoundsTest : TestWithParam<AgentBounds>
    {
    };

    TEST_P(DetourNavigatorNavigatorNotSupportedAgentBoundsTest, on_add_agent)
    {
        const Settings settings = makeSettings();
        NavigatorImpl navigator(settings, nullptr);
        EXPECT_FALSE(navigator.addAgent(GetParam()));
    }

    const std::array notSupportedAgentBounds = {
        AgentBounds{ .mShapeType = CollisionShapeType::Aabb, .mHalfExtents = osg::Vec3f(0, 0, 0) },
        AgentBounds{ .mShapeType = CollisionShapeType::RotatingBox, .mHalfExtents = osg::Vec3f(0, 0, 0) },
        AgentBounds{ .mShapeType = CollisionShapeType::Cylinder, .mHalfExtents = osg::Vec3f(0, 0, 0) },
        AgentBounds{ .mShapeType = CollisionShapeType::Aabb, .mHalfExtents = osg::Vec3f(0, 0, 11.34f) },
        AgentBounds{ .mShapeType = CollisionShapeType::RotatingBox, .mHalfExtents = osg::Vec3f(0, 11.34f, 11.34f) },
        AgentBounds{ .mShapeType = CollisionShapeType::Cylinder, .mHalfExtents = osg::Vec3f(0, 0, 11.34f) },
        AgentBounds{ .mShapeType = CollisionShapeType::Aabb, .mHalfExtents = osg::Vec3f(1, 1, 0) },
        AgentBounds{ .mShapeType = CollisionShapeType::RotatingBox, .mHalfExtents = osg::Vec3f(1, 1, 0) },
        AgentBounds{ .mShapeType = CollisionShapeType::Cylinder, .mHalfExtents = osg::Vec3f(1, 1, 0) },
        AgentBounds{ .mShapeType = CollisionShapeType::Aabb, .mHalfExtents = osg::Vec3f(1, 1, 11.33f) },
        AgentBounds{ .mShapeType = CollisionShapeType::RotatingBox, .mHalfExtents = osg::Vec3f(1, 1, 11.33f) },
        AgentBounds{ .mShapeType = CollisionShapeType::Cylinder, .mHalfExtents = osg::Vec3f(1, 1, 11.33f) },
        AgentBounds{ .mShapeType = CollisionShapeType::Aabb, .mHalfExtents = osg::Vec3f(2043.54f, 2043.54f, 11.34f) },
        AgentBounds{ .mShapeType = CollisionShapeType::RotatingBox, .mHalfExtents = osg::Vec3f(2890, 1, 11.34f) },
        AgentBounds{ .mShapeType = CollisionShapeType::Cylinder, .mHalfExtents = osg::Vec3f(2890, 2890, 11.34f) },
    };

    INSTANTIATE_TEST_SUITE_P(NotSupportedAgentBounds, DetourNavigatorNavigatorNotSupportedAgentBoundsTest,
        ValuesIn(notSupportedAgentBounds));
}
