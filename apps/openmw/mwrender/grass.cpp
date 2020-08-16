#include "grass.hpp"

#include <components/misc/stringops.hpp>

#include <components/sceneutil/lightmanager.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>

#include <components/shader/shadervisitor.hpp>
#include <components/shader/shadermanager.hpp>

#include <components/settings/settings.hpp>

#include "../mwmechanics/actorutil.hpp"

#include "vismask.hpp"

namespace MWRender
{
    void Grass::update()
    {
        osg::Vec3f playerPos(MWMechanics::getPlayer().getRefData().getPosition().asVec3());
        for (MWRender::GrassItem& item : mItems)
        {
            item.updateVisibility(playerPos);
        }

        const static bool useAnimation = Settings::Manager::getBool("animation", "Grass");
        if (!useAnimation)
            return;

        mWindSpeedUniform->set((osg::Vec3f) MWBase::Environment::get().getWorld()->getSmoothedStormDirection());
    }

   void Grass::insertGrass(osg::Group* cellnode, Resource::ResourceSystem* rs)
    {
        osg::ref_ptr<osg::Group> grassGroup = new osg::Group();
        grassGroup->setName("CellGrass");
        cellnode->addChild(grassGroup);

        for (MWRender::GrassItem& item : mItems)
        {
            attachToNode(item, grassGroup, rs);
        }

        osg::ref_ptr<Shader::ShaderVisitor> shaderVisitor (new Shader::ShaderVisitor(rs->getSceneManager()->getShaderManager(), *(rs->getImageManager()), "grass_vertex.glsl", "grass_fragment.glsl"));
        bool forceShaders = Settings::Manager::getBool("radial fog", "Shaders") || Settings::Manager::getBool("force shaders", "Shaders") || Settings::Manager::getBool("enable shadows", "Shadows");
        shaderVisitor->setForceShaders(forceShaders);
        grassGroup->accept(*shaderVisitor);
    }

    void Grass::attachToNode(MWRender::GrassItem& item, osg::Group* cellnode, Resource::ResourceSystem* rs)
    {
        osg::ref_ptr<SceneUtil::PositionAttitudeTransform> insert (new SceneUtil::PositionAttitudeTransform);
        cellnode->addChild(insert);

        insert->setPosition(item.mPos.asVec3());
        insert->setScale(osg::Vec3f(item.mScale, item.mScale, item.mScale));
        insert->setAttitude(
            osg::Quat(item.mPos.rot[2], osg::Vec3(0, 0, -1)) *
            osg::Quat(item.mPos.rot[1], osg::Vec3(0, -1, 0)) *
            osg::Quat(item.mPos.rot[0], osg::Vec3(-1, 0, 0)));

        insert->setNodeMask(Mask_Grass);

        rs->getSceneManager()->getInstance("meshes\\" + item.mModel, insert);

        const static bool useAnimation = Settings::Manager::getBool("animation", "Grass");
        if(useAnimation)
        {
            osg::StateSet* stateset = insert->getOrCreateStateSet();
            // for some reason this uniform is added to other objects too? not only for grass
            stateset->addUniform(new osg::Uniform("Rotz", (float) item.mPos.rot[2]));
            stateset->addUniform(mWindSpeedUniform.get());
        }

        item.mNode = insert;
    }

    bool Grass::isGrassItem(const std::string& model)
    {
        std::string mesh = Misc::StringUtils::lowerCase (model);
        if (mesh.find("grass\\") == 0)
            return true;

        return false;
    }

    bool Grass::isEnabled(const std::string& model)
    {
        static const float density = Settings::Manager::getFloat("density", "Grass");

        mCurrentGrass += density;
        if (mCurrentGrass < 1.f)
            return false;

        mCurrentGrass -= 1.f;
        return true;
    }

    bool Grass::loadGrassItem(const std::string& model, ESM::Position& pos, const float scale)
    {
        if (isGrassItem(model))
        {
            if (isEnabled(model))
            {
                GrassItem grass;
                grass.mModel = model;
                grass.mPos = pos;
                grass.mScale = scale;
                mItems.push_back(grass);
            }

            return true;
        }

        return false;
    }

    void GrassItem::updateVisibility(osg::Vec3f& playerPos)
    {
        static const int grassDistance = Settings::Manager::getInt("distance", "Grass");
        if (grassDistance <= 0) return;

        osg::Vec3f grassPos(mPos.asVec3());
        float dist = (playerPos - grassPos).length();
        if (mNode == nullptr) return;

        if (dist > grassDistance)
        {
            mNode->setNodeMask(0);
            return;
        }
        else
            mNode->setNodeMask(Mask_Grass);
    }
}
