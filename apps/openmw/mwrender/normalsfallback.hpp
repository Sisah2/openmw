#ifndef OPENMW_MWRENDER_NORMALS_FALLBACK_H
#define OPENMW_MWRENDER_NORMALS_FALLBACK_H

#include <array>
#include <osg/ref_ptr>

#include <osgUtil/RenderBin>
#include <osgUtil/IncrementalCompileOperation>

#include <osgViewer/Viewer>

namespace osg
{
    class Group;
//    class FrameBufferObject;
}

namespace MWRender
{
    class PostProcessor;
    class NormalsFallbackCamera;

    class CopyTextureCallback : public osg::Drawable::DrawCallback
    {
    public:
        CopyTextureCallback(osg::ref_ptr<PostProcessor> postProcessor);

        void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const override;

    private:
        osg::ref_ptr<PostProcessor> mPostProcessor;
        osg::ref_ptr<osg::StateSet> mStateSet;
    };

    // Postprocessing normals fallback
    class NormalsFallback
    {
        osg::ref_ptr<osg::Group> mRootNode;
        osg::ref_ptr<osg::Group> mSceneRoot;
        osg::ref_ptr<NormalsFallbackCamera> mNormalsFallbackCamera;
        osg::ref_ptr<PostProcessor> mPostProcessor;
        osg::ref_ptr<osg::Geometry> mNormalsFallbackGeometry;
        osg::ref_ptr<CopyTextureCallback> mCopyTextureCallback;
        bool mUseCamera;

    public:
        NormalsFallback(osg::Group* rootNode, osg::Group* sceneRoot, osg::ref_ptr<PostProcessor> postProcessor, osgUtil::IncrementalCompileOperation* ico, bool useCamera);

        ~NormalsFallback();

        void enable();
        void disable();
    };

}

#endif
