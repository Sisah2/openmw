#ifndef OPENMW_COMPONENTS_NIFBULLET_BULLETNIFLOADER_HPP
#define OPENMW_COMPONENTS_NIFBULLET_BULLETNIFLOADER_HPP

#include <cassert>
#include <string>
#include <set>
#include <map>

#include <osg/Matrixf>
#include <osg/BoundingBox>
#include <osg/ref_ptr>
#include <osg/Referenced>

#include <BulletCollision/CollisionShapes/btCompoundShape.h>

#include <components/debug/debuglog.hpp>
#include <components/nif/niffile.hpp>
#include <components/resource/bulletshape.hpp>

class btTriangleMesh;
class btCompoundShape;
class btCollisionShape;

namespace Nif
{
    struct Node;
    struct Transformation;
    struct NiTriShape;
    struct NiTriStrips;
    struct NiGeometry;
}

namespace NifBullet
{

/**
*Load bulletShape from NIF files.
*/
class BulletNifLoader
{
public:
    void warn(const std::string &msg)
    {
        Log(Debug::Warning) << "NIFLoader: Warn: " << msg;
    }

    [[noreturn]] void fail(const std::string &msg)
    {
        Log(Debug::Error) << "NIFLoader: Fail: "<< msg;
        abort();
    }

    osg::ref_ptr<Resource::BulletShape> load(const Nif::File& file);

private:
    bool findBoundingBox(const Nif::Node& node, const std::string& filename);

    void handleNode(const std::string& fileName, const Nif::Node& node, int flags, bool isCollisionNode,
                    bool isAnimated=false, bool autogenerated=false, bool avoid=false);

    bool hasAutoGeneratedCollision(const Nif::Node& rootNode);

    void handleNiTriShape(const Nif::Node& nifNode, int flags, const osg::Matrixf& transform, bool isAnimated, bool avoid);

    void handleNiTriShape(const Nif::NiGeometry& nifNode, const osg::Matrixf& transform, bool isAnimated, bool avoid);

    std::unique_ptr<btCompoundShape, Resource::DeleteCollisionShape> mCompoundShape;

    std::unique_ptr<btTriangleMesh> mStaticMesh;

    std::unique_ptr<btTriangleMesh> mAvoidStaticMesh;

    osg::ref_ptr<Resource::BulletShape> mShape;
};

}

#endif
