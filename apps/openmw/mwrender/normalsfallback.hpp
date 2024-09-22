#ifndef OPENMW_MWRENDER_NORMALS_FALLBACK_H
#define OPENMW_MWRENDER_NORMALS_FALLBACK_H

#include <osg/ref_ptr>

namespace osg
{
    class Group;
}

namespace MWRender
{

    class NormalsFallbackCamera;
    class PostProcessor;

    /// Postprocessing normals fallback
    class NormalsFallback
    {
        osg::ref_ptr<osg::Group> mParent;
        osg::ref_ptr<osg::Group> mSceneRoot;
        osg::ref_ptr<NormalsFallbackCamera> mNormalsFallbackCamera;
        osg::ref_ptr<PostProcessor> mPostProcessor;

    public:
        NormalsFallback(osg::Group* parent, osg::Group* sceneRoot, osg::ref_ptr<PostProcessor> postProcessor);

        ~NormalsFallback();

        void enable();
        void disable();
    };

}

#endif
