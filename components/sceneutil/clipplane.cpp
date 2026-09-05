#include "clipplane.hpp"

#include <format>

#include <osg/StateSet>

#include "glextensions.hpp"

namespace
{
    std::string getClipPlaneUniform(unsigned int index)
    {
        return std::format("clipPlane{}", index);
    }
}

namespace SceneUtil
{
    void updateClipPlane(osg::StateSet& stateset, unsigned int index, const osg::Vec4f& plane)
    {
        stateset.getOrCreateUniform(getClipPlaneUniform(index), osg::Uniform::FLOAT_VEC4)->set(plane);
    }

    void setClipPlaneMode(osg::StateSet& stateset, unsigned int index, osg::StateAttribute::OverrideValue value)
    {
        if (index >= NumClipPlanes)
            throw std::out_of_range(std::format("Clip plane #{} exceeds maximum of {}", index, NumClipPlanes));

        // GL_CLIP_DISTANCEi and GL_CLIP_PLANEi are aliases. This means we can set the same modes for FFP clip
        // planes and forward compatible vertex clipping
#if defined(OSG_GLES2_AVAILABLE) || defined(OSG_GLES3_AVAILABLE)
        if (supportsNativeClipDistance())
#endif
            stateset.setMode(GL_CLIP_PLANE0 + index, value);

#if defined(OSG_GLES2_AVAILABLE) || defined(OSG_GLES3_AVAILABLE)
        // There is a driver bug where writing to gl_ClipDistance does not respect the current modes
        // We must instead control clip plane writes via uniform
        osg::Uniform* uniform
            = stateset.getOrCreateUniform(std::format("clipPlaneEnabled{}", index), osg::Uniform::BOOL);
        uniform->set((value & osg::StateAttribute::ON) != 0);
        stateset.addUniform(uniform, value | osg::StateAttribute::ON);
#endif
    }
}
