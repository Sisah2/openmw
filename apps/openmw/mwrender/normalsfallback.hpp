#ifndef OPENMW_MWRENDER_NORMALS_FALLBACK_H
#define OPENMW_MWRENDER_NORMALS_FALLBACK_H

#include <array>
#include <osg/ref_ptr>

#include <osgUtil/RenderBin>

#include <osgViewer/Viewer>

namespace osg
{
    class Group;
    class FrameBufferObject;
}

namespace MWRender
{
    class PostProcessor;
    class NormalsFallbackCamera;

    class DecodeNormalsCallback : public osgUtil::RenderBin::DrawCallback
    {
    public:
        void drawImplementation(
            osgUtil::RenderBin* bin, osg::RenderInfo& renderInfo, osgUtil::RenderLeaf*& previous) override;

        void setFBO(const osg::ref_ptr<osg::FrameBufferObject>& fbo, size_t frameId) { mFBO[frameId] = fbo; }

        void setOriginalFBO(const osg::ref_ptr<osg::FrameBufferObject>& fbo, size_t frameId) { mOriginalFBO[frameId] = fbo; }

        void setStateset(osg::ref_ptr<osg::StateSet> stateSet) { mStateSet = stateSet; }
    private:
        std::array<osg::ref_ptr<osg::FrameBufferObject>, 2> mFBO;
        std::array<osg::ref_ptr<osg::FrameBufferObject>, 2> mOriginalFBO;
        osg::ref_ptr<osg::StateSet> mStateSet;
    };

    // Postprocessing normals fallback
    class NormalsFallback
    {
        osg::ref_ptr<osg::Group> mParent;
        osg::ref_ptr<osg::Group> mSceneRoot;
        osg::ref_ptr<osg::Group> mRootNode;
        osg::ref_ptr<NormalsFallbackCamera> mNormalsFallbackCamera;
        osg::ref_ptr<PostProcessor> mPostProcessor;
        osg::ref_ptr<DecodeNormalsCallback> mDecodeNormalsCallback;
        osg::ref_ptr<osg::Geode> mNormalsFallbackGeode;

        std::array<osg::ref_ptr<osg::FrameBufferObject>, 2> mFBO;
        std::array<osg::ref_ptr<osg::FrameBufferObject>, 2> mOriginalFBO;

    public:
        NormalsFallback(osg::Group* parent, osg::Group* sceneRoot, osg::Group* rootNode, osgViewer::Viewer* viewer, osg::ref_ptr<PostProcessor> postProcessor);

        ~NormalsFallback();

        void setFBO(const osg::ref_ptr<osg::FrameBufferObject>& fbo, size_t frameId)
        {
            mFBO[frameId] = fbo; 
            mDecodeNormalsCallback->setFBO(mFBO[frameId], frameId);
        }

        void setOriginalFBO(const osg::ref_ptr<osg::FrameBufferObject>& fbo, size_t frameId) 
        {
            mOriginalFBO[frameId] = fbo; 
            mDecodeNormalsCallback->setOriginalFBO(mOriginalFBO[frameId], frameId);
        }

        void enable();
        void disable();
    };

}

#endif
