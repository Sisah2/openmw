#include "normalsfallback.hpp"

#include <osg/Group>
#include <osg/BlendFunc>

//#include <osgUtil/IncrementalCompileOperation>

#include <components/resource/resourcesystem.hpp>
#include <components/resource/scenemanager.hpp>
#include <components/sceneutil/rtt.hpp>
#include <components/sceneutil/shadow.hpp>
#include <components/shader/shadermanager.hpp>
#include <components/settings/values.hpp>
#include <components/debug/debuglog.hpp>

#include "util.hpp"
#include "vismask.hpp"
#include "renderbin.hpp"

#include "apps/openmw/mwbase/environment.hpp"
#include "postprocessor.hpp"
#include "camera.hpp"

#include "../mwbase/world.hpp"

namespace MWRender
{

    CopyTextureCallback::CopyTextureCallback(osg::ref_ptr<PostProcessor> postProcessor)
        : mPostProcessor(postProcessor)
        , mStateSet(new osg::StateSet)
    {
        mStateSet->setDefine("DECODE", "0", osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    }

    void CopyTextureCallback::drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const
    {
        osg::State& state = *renderInfo.getState();
        size_t frameId = state.getFrameStamp()->getFrameNumber() % 2;

        const auto& fbo = mPostProcessor->getFbo(PostProcessor::FBO_Primary, frameId);
        const auto& opaqueFbo = mPostProcessor->getFbo(PostProcessor::FBO_OpaqueDepth, frameId);

        state.applyTextureAttribute(0, mPostProcessor->getTexture(PostProcessor::Tex_Scene, frameId));

        opaqueFbo->apply(state, osg::FrameBufferObject::DRAW_FRAMEBUFFER);

        glClear(GL_COLOR_BUFFER_BIT);

        drawable->drawImplementation(renderInfo);

        fbo->apply(state, osg::FrameBufferObject::DRAW_FRAMEBUFFER);

// this part dont work it seems :(
state.pushStateSet(mStateSet);
state.apply();

drawable->drawImplementation(renderInfo);

state.popStateSet();
    }

    class NormalsFallbackCamera : public SceneUtil::RTTNode
    {
    public:
        NormalsFallbackCamera(osg::ref_ptr<PostProcessor> postProcessor, osg::Node* scene)
            : RTTNode(postProcessor->renderWidth(), postProcessor->renderHeight(), 0, false, 0, StereoAwareness::Aware, shouldAddMSAAIntermediateTarget())
        {
            setColorBufferInternalFormat(GL_RGB);
            setDepthBufferInternalFormat(GL_DEPTH24_STENCIL8);
            mPostProcessor = postProcessor;
            mScene = scene;
            mNormalsFallbackRenderingDistance = std::min(Settings::postProcessing().mCameraNormalsFallbackRenderingDistance, Settings::camera().mViewingDistance);
        }

        void setDefaults(osg::Camera* camera) override
        {

            if (mNormalsFallbackRenderingDistance > 0)
                camera->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
            else
                camera->setReferenceFrame(osg::Camera::RELATIVE_RF);

            camera->setName("Normals Fallback Camera");

            camera->addChild(mScene);
            camera->setNodeMask(Mask_RenderToTexture);

            osg::Camera::CullingMode cullingMode = osg::Camera::DEFAULT_CULLING | osg::Camera::FAR_PLANE_CULLING;

            if (!Settings::camera().mSmallFeatureCulling)
                cullingMode &= ~(osg::CullStack::SMALL_FEATURE_CULLING);
            else
            {
                camera->setSmallFeatureCullingPixelSize(Settings::camera().mSmallFeatureCullingPixelSize);
                cullingMode |= osg::CullStack::SMALL_FEATURE_CULLING;
            }

            camera->setComputeNearFarMode(osg::Camera::DO_NOT_COMPUTE_NEAR_FAR);
            camera->setCullingMode(cullingMode);
            camera->setCullingActive(true);

            camera->setCullMask(Mask_Scene | Mask_Object | Mask_Static
                | Mask_Terrain | Mask_Actor | Mask_Player 
                | Mask_Groundcover | Mask_Water | Mask_SimpleWater);


            if (mNormalsFallbackRenderingDistance > 0)
            {
                const double width = Settings::video().mResolutionX;
                const double height = Settings::video().mResolutionY;
                double aspect = (height == 0.0) ? 1.0 : width / height;
                camera->setProjectionMatrixAsPerspective(Settings::camera().mFieldOfView, aspect, Settings::camera().mNearClip, mNormalsFallbackRenderingDistance);

                if (SceneUtil::AutoDepth::isReversed())
                    mProjectionMatrix = static_cast<osg::Matrixf>(SceneUtil::getReversedZProjectionMatrixAsPerspective(
Settings::camera().mFieldOfView, aspect, Settings::camera().mNearClip, mNormalsFallbackRenderingDistance));
                else
                    mProjectionMatrix = camera->getProjectionMatrix();

/*
                camera->getOrCreateStateSet()->addUniform(new osg::Uniform("projectionMatrix", static_cast<osg::Matrixf>(mProjectionMatrix)),
                    osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
*/
            }

            std::map<std::string, std::string> defineMap;
            Shader::ShaderManager& shaderMgr = MWBase::Environment::get().getResourceSystem()->getSceneManager()->getShaderManager();
            osg::ref_ptr<osg::Program> program = shaderMgr.getProgram("normal", defineMap);
            camera->getOrCreateStateSet()->setAttributeAndModes(program.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

            SceneUtil::ShadowManager::instance().disableShadowsForStateSet(*camera->getOrCreateStateSet());
        }

        void apply(osg::Camera* camera) override
        {
            mColorTexture = camera->getBufferAttachmentMap()[osg::Camera::COLOR_BUFFER]._texture;
            mPostProcessor->setExternalNormalsTexture(mColorTexture);

            if (mNormalsFallbackRenderingDistance > 0)
            {
                camera->setProjectionMatrix(mProjectionMatrix);
                camera->setViewMatrix(MWBase::Environment::get().getWorld()->getCamera()->getViewMatrix());
            }
        }

    private:
        osg::ref_ptr<osg::Node> mScene;
        osg::ref_ptr<PostProcessor> mPostProcessor;
        osg::ref_ptr<osg::Texture> mColorTexture;
        osg::Matrixf mProjectionMatrix;
        int mNormalsFallbackRenderingDistance;
    };

    NormalsFallback::NormalsFallback(osg::Group* rootNode, osg::Group* sceneRoot, osg::ref_ptr<PostProcessor> postProcessor, osgUtil::IncrementalCompileOperation* ico, bool useCamera)
        : mRootNode(rootNode)
        , mSceneRoot(sceneRoot)
        , mPostProcessor(postProcessor)
        , mUseCamera(useCamera)
    {
        if (!mUseCamera)
        {
            Shader::ShaderManager& shaderMgr = MWBase::Environment::get().getResourceSystem()->getSceneManager()->getShaderManager();
            Shader::ShaderManager::DefineMap defines;

            mCopyTextureCallback = new CopyTextureCallback(mPostProcessor);

            osg::ref_ptr<osg::Depth> depth = new SceneUtil::AutoDepth;
            depth->setWriteMask(false);

            osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array;
            verts->push_back(osg::Vec3f(-1, -1, 0));
            verts->push_back(osg::Vec3f(-1, 3, 0));
            verts->push_back(osg::Vec3f(3, -1, 0));

            mNormalsFallbackGeometry = new osg::Geometry;
            mNormalsFallbackGeometry->setUseDisplayList(false);
            mNormalsFallbackGeometry->setUseVertexBufferObjects(true);
            mNormalsFallbackGeometry->setVertexArray(verts);
            mNormalsFallbackGeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, 3));
            mNormalsFallbackGeometry->setDrawCallback(mCopyTextureCallback);
            mNormalsFallbackGeometry->setCullingActive(false);

            osg::ref_ptr<osg::StateSet> stateSet = new osg::StateSet;
            stateSet->setRenderBinDetails(MWRender::RenderBin_DepthSorted - 1, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
            stateSet->setAttributeAndModes(shaderMgr.getProgram("fullscreen_tri", defines), osg::StateAttribute::ON);
            stateSet->setAttributeAndModes(new osg::BlendFunc, osg::StateAttribute::OFF);
            stateSet->setAttributeAndModes(depth, osg::StateAttribute::ON);
            stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
            stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
            stateSet->addUniform(new osg::Uniform("lastShader", 0), osg::StateAttribute::ON);
            stateSet->addUniform(new osg::Uniform("scaling", osg::Vec2f(1, 1)), osg::StateAttribute::ON);
            stateSet->setDefine("DECODE", "1", osg::StateAttribute::ON);

            mNormalsFallbackGeometry->setStateSet(stateSet);
        }
    }														

    NormalsFallback::~NormalsFallback()
    {
        if (mNormalsFallbackCamera)
        {
            mRootNode->removeChild(mNormalsFallbackCamera);
            mNormalsFallbackCamera = nullptr;
        }

        mRootNode->removeChild(mNormalsFallbackGeometry);
    }

    void NormalsFallback::enable()
    {
        if (mUseCamera)
        {
            if (!mNormalsFallbackCamera)
                mNormalsFallbackCamera = new NormalsFallbackCamera(mPostProcessor, mSceneRoot);

            mRootNode->addChild(mNormalsFallbackCamera);
        }
        else
           mRootNode->addChild(mNormalsFallbackGeometry);

    }

    void NormalsFallback::disable()
    {
        if (mUseCamera)
        {
            mRootNode->removeChild(mNormalsFallbackCamera);
            mPostProcessor->setExternalNormalsTexture(nullptr);
            mNormalsFallbackCamera = nullptr;
        }
        else
            mRootNode->removeChild(mNormalsFallbackGeometry);
    }
}


