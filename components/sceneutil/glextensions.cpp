#include "glextensions.hpp"

#include <osg/GraphicsContext>

namespace SceneUtil
{
    namespace
    {
        std::set<osg::observer_ptr<osg::GLExtensions>> sGLExtensions;

        class GLExtensionsObserver : public osg::Observer
        {
        public:
            static GLExtensionsObserver sInstance;

            ~GLExtensionsObserver() override
            {
                for (auto& ptr : sGLExtensions)
                {
                    osg::ref_ptr<osg::GLExtensions> ref;
                    if (ptr.lock(ref))
                        ref->removeObserver(this);
                }
            }

            void objectDeleted(void* referenced) override
            {
                sGLExtensions.erase(static_cast<osg::GLExtensions*>(referenced));
            }
        };

        // construct after sGLExtensions so this gets destroyed first.
        GLExtensionsObserver GLExtensionsObserver::sInstance{};
    }

    bool glExtensionsReady()
    {
        return !sGLExtensions.empty();
    }

    osg::GLExtensions& getGLExtensions()
    {
        if (sGLExtensions.empty())
            throw std::runtime_error(
                "GetGLExtensionsOperation was not used when the current context was created or there is no current "
                "context");
        return **sGLExtensions.begin();
    }

    bool supportsNativeClipDistance()
    {
#if defined(OSG_GLES2_AVAILABLE) || defined(OSG_GLES3_AVAILABLE)
        return osg::isGLExtensionSupported(getGLExtensions().contextID, "GL_EXT_clip_cull_distance");
#endif
        return getGLExtensions().glVersion >= 3.0f && getGLExtensions().glslLanguageVersion >= 3.3f;
    }

    bool useFixedFunctionClipPlanes()
    {
#if defined(OSG_GLES2_AVAILABLE) || defined(OSG_GLES3_AVAILABLE)
        return false;
#endif
        return !supportsNativeClipDistance();
    }

    GetGLExtensionsOperation::GetGLExtensionsOperation()
        : GraphicsOperation("GetGLExtensionsOperation", false)
    {
    }

    void GetGLExtensionsOperation::operator()(osg::GraphicsContext* graphicsContext)
    {
        auto [itr, _] = sGLExtensions.emplace(graphicsContext->getState()->get<osg::GLExtensions>());
        (*itr)->addObserver(&GLExtensionsObserver::sInstance);
    }
}
