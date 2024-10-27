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
    void DecodeNormalsCallback::drawImplementation(
        osgUtil::RenderBin* bin, osg::RenderInfo& renderInfo, osgUtil::RenderLeaf*& previous)
    {
        osg::State* state = renderInfo.getState();
        size_t frameId = state->getFrameStamp()->getFrameNumber() % 2;

        PostProcessor* postProcessor = dynamic_cast<PostProcessor*>(renderInfo.getCurrentCamera()->getUserData());

        if (!postProcessor || bin->getStage()->getFrameBufferObject() != postProcessor->getPrimaryFbo(frameId))
            return;

        constexpr osg::StateAttribute::OverrideValue modeOn = osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE;
        mStateSet->setTextureAttributeAndModes(0, postProcessor->getTexture(PostProcessor::Tex_Scene, frameId), modeOn);
        mStateSet->setTextureAttributeAndModes(1, postProcessor->getTexture(PostProcessor::Tex_Normal, frameId), modeOn);

        //mFBO[frameId]->apply(*state, osg::FrameBufferObject::DRAW_FRAMEBUFFER);
        //mOriginalFBO[frameId]->apply(*state, osg::FrameBufferObject::READ_FRAMEBUFFER);

        const osg::Texture* tex
            = mFBO[frameId]->getAttachment(osg::FrameBufferObject::BufferComponent::COLOR_BUFFER0).getTexture();

        auto* resolveViewport = state->getCurrentViewport();
        state->pushStateSet(mStateSet);
        state->apply();
/*        resolveViewport->apply(*state);
        glViewport(0, 0, tex->getTextureWidth(), tex->getTextureHeight());
        state->haveAppliedAttribute(osg::StateAttribute::VIEWPORT);

        glClearColor(0.0, 1.0, 0.0, 1.0);
        glColorMask(true, true, true, true);
        state->haveAppliedAttribute(osg::StateAttribute::Type::COLORMASK);
*/
        //state->applyTextureAttribute(0, postProcessor->getTexture(PostProcessor::Tex_Scene, frameId));
        //state->applyTextureAttribute(1, postProcessor->getTexture(PostProcessor::Tex_Normal, 0));

        mFBO[frameId]->apply(*state, osg::FrameBufferObject::DRAW_FRAMEBUFFER);
        mOriginalFBO[frameId]->apply(*state, osg::FrameBufferObject::READ_FRAMEBUFFER);

        glClear(GL_COLOR_BUFFER_BIT);

        Log(Debug::Error) << "DecodeNormalsCallback::drawImplementation";

        bin->drawImplementation(renderInfo, previous);

        state->popStateSet();

        tex = mOriginalFBO[frameId]->getAttachment(osg::FrameBufferObject::BufferComponent::COLOR_BUFFER0).getTexture();
        glViewport(0, 0, tex->getTextureWidth(), tex->getTextureHeight());
        mOriginalFBO[frameId]->apply(*state, osg::FrameBufferObject::DRAW_FRAMEBUFFER);
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
            mNormalsFallbackRenderingDistance = Settings::postProcessing().mNormalsFallbackRenderingDistance;
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
            camera->setCullingActive(/*true*/false);
/*
            if (camera->getBufferAttachmentMap().count(osg::Camera::COLOR_BUFFER) == 0)
            {
                camera->attach(osg::Camera::COLOR_BUFFER, mColorTexture, 0, 0, false, 1);
                SceneUtil::attachAlphaToCoverageFriendlyFramebufferToCamera(camera, osg::Camera::COLOR_BUFFER,
                    mColorTexture, 0, 0, false, MWRender::shouldAddMSAAIntermediateTarget());
            }

            if (camera->getBufferAttachmentMap().count(osg::Camera::PACKED_DEPTH_STENCIL_BUFFER) == 0)
                camera->attach(osg::Camera::PACKED_DEPTH_STENCIL_BUFFER, mDepthTexture, 0, 0, false, 1);
*/

            if (Settings::postProcessing().mNormalsFallbackMode == 1 || Settings::postProcessing().mTest1 == 1)
                camera->setCullMask(Mask_Scene | Mask_Object | Mask_Static
                    | Mask_Terrain | Mask_Actor | Mask_Player 
                    | Mask_Groundcover | Mask_Water | Mask_SimpleWater/* | Mask_FirstPerson*/);
            else
            {
                camera->setCullMask(Mask_Scene | Mask_Terrain /*| Mask_Water */| Mask_SimpleWater);
                camera->setClearColor(osg::Vec4f(0.0, 0.0, 1.0, 1.0));
            }

            if (mNormalsFallbackRenderingDistance > 0){
                const double width = Settings::video().mResolutionX;
                const double height = Settings::video().mResolutionY;
                double aspect = (height == 0.0) ? 1.0 : width / height;
                camera->setProjectionMatrixAsPerspective(Settings::camera().mFieldOfView, aspect, Settings::camera().mNearClip,
                    Settings::postProcessing().mNormalsFallbackRenderingDistance);

                if (SceneUtil::AutoDepth::isReversed())
                    mProjectionMatrix = static_cast<osg::Matrixf>(SceneUtil::getReversedZProjectionMatrixAsPerspective(
Settings::camera().mFieldOfView, aspect, Settings::camera().mNearClip, Settings::postProcessing().mNormalsFallbackRenderingDistance));
                else
                    mProjectionMatrix = camera->getProjectionMatrix();


                if (Settings::postProcessing().mTest2 == 1)
                camera->getOrCreateStateSet()->addUniform(new osg::Uniform("projectionMatrix", static_cast<osg::Matrixf>(mProjectionMatrix)),
                    osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
            }


            if (Settings::postProcessing().mDisableBlending)
                camera->getOrCreateStateSet()->setAttributeAndModes(new osg::BlendFunc, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

            camera->getOrCreateStateSet()->setDefine("NORMALS_FALLBACK", "1", osg::StateAttribute::ON);

            if (Settings::postProcessing().mUseSimplifiedShader)
            {
                std::map<std::string, std::string> defineMap;
                Shader::ShaderManager& shaderMgr = MWBase::Environment::get().getResourceSystem()->getSceneManager()->getShaderManager();
                osg::ref_ptr<osg::Program> program = shaderMgr.getProgram("normal", defineMap);

                camera->getOrCreateStateSet()->setAttributeAndModes(program.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
            }

            SceneUtil::ShadowManager::instance().disableShadowsForStateSet(*camera->getOrCreateStateSet());
        }

        void apply(osg::Camera* camera) override
        {

            mColorTexture = camera->getBufferAttachmentMap()[osg::Camera::COLOR_BUFFER]._texture;
            mPostProcessor->setExternalNormalsTexture(mColorTexture);

            mDepthTexture = camera->getBufferAttachmentMap()[osg::Camera::PACKED_DEPTH_STENCIL_BUFFER]._texture;
            mPostProcessor->setExternalDepthTexture(mDepthTexture);

            if (mNormalsFallbackRenderingDistance > 0)
            {
                camera->setProjectionMatrix(mProjectionMatrix);
                camera->setViewMatrix(MWBase::Environment::get().getWorld()->getCamera()->getViewMatrix());

//mPostProcessor->getStateUpdater()->setProjectionMatrix(mProjectionMatrix);

            }
        }

        void setTextures(osg::ref_ptr<osg::Texture> color, osg::ref_ptr<osg::Texture> depth)
        {
            mColorTexture = color;
            mDepthTexture = depth;
        }

    private:
        osg::ref_ptr<osg::Node> mScene;
        osg::ref_ptr<PostProcessor> mPostProcessor;
        osg::ref_ptr<osg::Texture> mColorTexture;
        osg::ref_ptr<osg::Texture> mDepthTexture;
        osg::Matrixf mProjectionMatrix;
        int mNormalsFallbackRenderingDistance;
    };

    NormalsFallback::NormalsFallback(osg::Group* parent, osg::Group* sceneRoot, osg::Group* rootNode, osgViewer::Viewer* viewer, osg::ref_ptr<PostProcessor> postProcessor)
        : mParent(parent)
        , mSceneRoot(sceneRoot)
        , mRootNode(rootNode)
        , mPostProcessor(postProcessor)
        , mDecodeNormalsCallback(new DecodeNormalsCallback)
    {

        if (Settings::postProcessing().mNormalsFallbackMode == 2 && Settings::postProcessing().mTest4 != 1)
        {
            constexpr osg::StateAttribute::OverrideValue modeOff = osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE;
            constexpr osg::StateAttribute::OverrideValue modeOn = osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE;

            osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array;
            verts->push_back(osg::Vec3f(-1, -1, 0));
            verts->push_back(osg::Vec3f(-1, 3, 0));
            verts->push_back(osg::Vec3f(3, -1, 0));

            osg::ref_ptr<osg::Geometry> normalsFallbackGeometry = new osg::Geometry;
            normalsFallbackGeometry->setUseDisplayList(false);
            normalsFallbackGeometry->setUseVertexBufferObjects(true);
            normalsFallbackGeometry->setVertexArray(verts);
            normalsFallbackGeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, 3));

            std::map<std::string, std::string> defines;
            Shader::ShaderManager& shaderMgr = MWBase::Environment::get().getResourceSystem()->getSceneManager()->getShaderManager();

            osg::StateSet* normalsFallbackStateSet = normalsFallbackGeometry->getOrCreateStateSet();
            normalsFallbackStateSet->setAttributeAndModes(shaderMgr.getProgram("fullscreen_tri", defines), modeOn);
            normalsFallbackStateSet->addUniform(new osg::Uniform("lastShader", 0), modeOn);
            normalsFallbackStateSet->addUniform(new osg::Uniform("externalNormals", 1), modeOn);
            normalsFallbackStateSet->addUniform(new osg::Uniform("scaling", osg::Vec2f(1, 1)));
            normalsFallbackStateSet->setRenderBinDetails(RenderBin_NormalsFallback, "NormalsFallback");
            normalsFallbackStateSet->setDefine("WRITE_NORMALS", "1", osg::StateAttribute::ON);
            normalsFallbackStateSet->setAttributeAndModes(new osg::BlendFunc, modeOff);
            normalsFallbackStateSet->setAttributeAndModes(new SceneUtil::AutoDepth, modeOff);
            normalsFallbackStateSet->setMode(GL_DEPTH_TEST,  osg::StateAttribute::OFF);
/*
            for (unsigned int unit = 2; unit < 8; ++unit)
                normalsFallbackStateSet->setTextureMode(unit, GL_TEXTURE_2D, modeOff);
*/
            osg::ref_ptr<osgUtil::RenderBin> normalsFallbackRenderBin
                = new osgUtil::RenderBin(osgUtil::RenderBin::SORT_BACK_TO_FRONT);

            mNormalsFallbackGeode = new osg::Geode();
            mNormalsFallbackGeode->addDrawable(normalsFallbackGeometry);
            mNormalsFallbackGeode->setCullingActive(/*true*/false);

            osg::StateSet* normalsFallbackGeodeStateSet = mNormalsFallbackGeode->getOrCreateStateSet();
            normalsFallbackGeodeStateSet->setAttributeAndModes(shaderMgr.getProgram("fullscreen_tri", defines), modeOn);
            normalsFallbackGeodeStateSet->addUniform(new osg::Uniform("lastShader", 0), modeOn);
            normalsFallbackGeodeStateSet->addUniform(new osg::Uniform("externalNormals", 1), modeOn);
            normalsFallbackGeodeStateSet->addUniform(new osg::Uniform("scaling", osg::Vec2f(1, 1)));
            normalsFallbackGeodeStateSet->setRenderBinDetails(RenderBin_NormalsFallback, "NormalsFallback");
            normalsFallbackGeodeStateSet->setDefine("WRITE_NORMALS", "1", osg::StateAttribute::ON);
            normalsFallbackGeodeStateSet->setAttributeAndModes(new osg::BlendFunc, modeOff);
            normalsFallbackGeodeStateSet->setAttributeAndModes(new SceneUtil::AutoDepth, modeOff);
            normalsFallbackGeodeStateSet->setMode(GL_DEPTH_TEST,  osg::StateAttribute::OFF);

            mSceneRoot->addChild(mNormalsFallbackGeode);
/*
            osg::ref_ptr<osg::Node> dummyNodeToClear = new osg::Node;
            dummyNodeToClear->setCullingActive(false);
            //dummyNodeToClear->addChild(normalsFallbackGeometry);
            dummyNodeToClear->getOrCreateStateSet()->setRenderBinDetails(RenderBin_NormalsFallback, "NormalsFallback");
            mSceneRoot->addChild(dummyNodeToClear);
*/

            mDecodeNormalsCallback->setStateset(normalsFallbackGeodeStateSet);
     //       normalsFallbackRenderBin->getStateSet()->addUniform(new osg::Uniform("lastShader", 0), modeOn);
    //        normalsFallbackRenderBin->getStateSet()->addUniform(new osg::Uniform("externalNormals", 1), modeOn);
            normalsFallbackRenderBin->setDrawCallback(mDecodeNormalsCallback);

            osgUtil::RenderBin::addRenderBinPrototype("NormalsFallback", normalsFallbackRenderBin);
        }
    }														

    NormalsFallback::~NormalsFallback()
    {
        if (mNormalsFallbackCamera)
        {
            mParent->removeChild(mNormalsFallbackCamera);
            mNormalsFallbackCamera = nullptr;
        }
    }

    void NormalsFallback::enable()
    {
        if (!mNormalsFallbackCamera)
            mNormalsFallbackCamera = new NormalsFallbackCamera(mPostProcessor, mSceneRoot);

     //   mNormalsFallbackCamera->setTextures(mPostProcessor->getTexture(PostProcessor::Tex_Normal, 0), mPostProcessor->getTexture(PostProcessor::Tex_OpaqueDepth, 0));

        mParent->addChild(mNormalsFallbackCamera);

     //   mRootNode->addChild(mNormalsFallbackGeode);
    //    mRootNode->addChild(normalsFallbackGeometry);

    }

    void NormalsFallback::disable()
    {
        mParent->removeChild(mNormalsFallbackCamera);
   //     mParent->removeChild(mNormalsFallbackGeode);
        mPostProcessor->setExternalNormalsTexture(nullptr);
        mPostProcessor->setExternalDepthTexture(nullptr);
        mNormalsFallbackCamera = nullptr;
													
    }
}
