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

#include "util.hpp"
#include "vismask.hpp"

#include "apps/openmw/mwbase/environment.hpp"
#include "postprocessor.hpp"

namespace MWRender
{
    /// This callback on the Camera has the effect of a RELATIVE_RF_INHERIT_VIEWPOINT transform mode (which does not
    /// exist in OSG). We want to keep the View Point of the parent camera so we will not have to recreate LODs.
    class InheritViewPointCallback
        : public SceneUtil::NodeCallback<InheritViewPointCallback, osg::Node*, osgUtil::CullVisitor*>
    {
    public:
        InheritViewPointCallback() {}

        void operator()(osg::Node* node, osgUtil::CullVisitor* cv)
        {
            osg::ref_ptr<osg::RefMatrix> modelViewMatrix = new osg::RefMatrix(*cv->getModelViewMatrix());
            cv->popModelViewMatrix();
            cv->pushModelViewMatrix(modelViewMatrix, osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT);
            traverse(node, cv);
        }
    };

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
        }

        void setDefaults(osg::Camera* camera) override
        {
            camera->setReferenceFrame(osg::Camera::RELATIVE_RF);
            camera->setSmallFeatureCullingPixelSize(Settings::camera().mSmallFeatureCullingPixelSize);
            camera->setName("Normals Fallback Camera");
//if (Settings::postProcessing().mTest3 == 0)
//            camera->addCullCallback(new InheritViewPointCallback);

            camera->addChild(mScene);
            camera->setNodeMask(Mask_RenderToTexture);
            camera->setClearColor(osg::Vec4f(0.f, 0.f, 0.f, 1.f));

if (Settings::postProcessing().mTest2 == 0)
            camera->getOrCreateStateSet()->setAttributeAndModes(new osg::BlendFunc, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

            if (Settings::postProcessing().mTest1 == 0)
            {
                std::map<std::string, std::string> defineMap;
                Shader::ShaderManager& shaderMgr = MWBase::Environment::get().getResourceSystem()->getSceneManager()->getShaderManager();
                osg::ref_ptr<osg::Program> program = shaderMgr.getProgram("normal", defineMap);
                camera->getOrCreateStateSet()->setAttributeAndModes(program.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
            }
            else
                camera->getOrCreateStateSet()->addUniform(new osg::Uniform("isNormalsFallback", true));

            SceneUtil::ShadowManager::instance().disableShadowsForStateSet(*camera->getOrCreateStateSet());
        }

        void apply(osg::Camera* camera) override
        {
            camera->setViewMatrix(osg::Matrix::identity());
            camera->setCullMask(Mask_Scene | Mask_Object | Mask_Static
            | Mask_Terrain | Mask_Actor | Mask_Player 
            | Mask_Groundcover | Mask_Water | Mask_SimpleWater/* | Mask_FirstPerson*/);


            if (Settings::postProcessing().mNormalsFallbackRenderingDistance != 0){
                const double width = mPostProcessor->renderWidth()/*Settings::video().mResolutionX*/;
                const double height = mPostProcessor->renderWidth()/*Settings::video().mResolutionY*/;
                double aspect = (height == 0.0) ? 1.0 : width / height;
                camera->setProjectionMatrixAsPerspective(Settings::camera().mFieldOfView, aspect, Settings::camera().mNearClip,
                    Settings::postProcessing().mNormalsFallbackRenderingDistance);


                osg::Matrixf projectionMatrix;

                if (SceneUtil::AutoDepth::isReversed())
                    projectionMatrix = SceneUtil::getReversedZProjectionMatrixAsPerspective(Settings::camera().mFieldOfView,
                        aspect, Settings::camera().mNearClip, Settings::postProcessing().mNormalsFallbackRenderingDistance);
                else
                    projectionMatrix = camera->getProjectionMatrix();
        

                camera->getOrCreateStateSet()->addUniform(new osg::Uniform("projectionMatrix", static_cast<osg::Matrixf>(projectionMatrix)),
                    osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

                mPostProcessor->getStateUpdater()->setProjectionMatrix(projectionMatrix);

            }

            mPostProcessor->setExternalNormalsTexture(camera->getBufferAttachmentMap()[osg::Camera::COLOR_BUFFER]._texture);
        }

    private:
        osg::ref_ptr<osg::Node> mScene;
        osg::ref_ptr<PostProcessor> mPostProcessor;
    };

    NormalsFallback::NormalsFallback(osg::Group* parent, osg::Group* sceneRoot, osg::ref_ptr<PostProcessor> postProcessor)
        : mParent(parent)
        , mSceneRoot(sceneRoot)
        , mPostProcessor(postProcessor)
    {
    //    mNormalsFallbackCamera = new NormalsFallbackCamera(mPostProcessor, mSceneRoot);
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

        mParent->addChild(mNormalsFallbackCamera);
    }

    void NormalsFallback::disable()
    {
        if (mNormalsFallbackCamera)
        {
            mParent->removeChild(mNormalsFallbackCamera);
            mNormalsFallbackCamera = nullptr;
        }
    }
}
