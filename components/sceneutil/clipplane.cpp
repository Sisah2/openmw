#include "clipplane.hpp"

#include <format>

#include <osg/StateSet>

namespace
{
    std::string getClipPlaneUniform(unsigned int index)
    {
        return std::format("clipPlane{}", index);
    }
}

namespace SceneUtil
{
    void setClipPlane(
        osg::StateSet& stateset, unsigned int index, const osg::Vec4f& plane, osg::StateAttribute::OverrideValue value)
    {
        stateset.addUniform(new osg::Uniform(getClipPlaneUniform(index).c_str(), plane), value);
    }

    void updateClipPlane(osg::StateSet& stateset, unsigned int index, const osg::Vec4f& plane)
    {
        stateset.getUniform(getClipPlaneUniform(index))->set(plane);
    }

    void setClipPlaneMode(osg::StateSet& stateset, unsigned int index, osg::StateAttribute::OverrideValue value)
    {
        if (index >= NumClipPlanes)
            throw std::out_of_range(std::format("Clip plane #{} exceeds maximum of {}", index, NumClipPlanes));

        // GL_CLIP_DISTANCEi and GL_CLIP_PLANEi are aliases
        // This means we can set the same modes for FFP clip planes and forward compatible vertex clipping
        stateset.setMode(GL_CLIP_PLANE0 + index, value);
    }
}
