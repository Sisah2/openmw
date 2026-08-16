#ifndef OPENMW_COMPONENTS_SCENEUTIL_CLIPPLANE_H
#define OPENMW_COMPONENTS_SCENEUTIL_CLIPPLANE_H

#include <osg/StateAttribute>

namespace osg
{
    class StateSet;
}

namespace SceneUtil
{
    // This number may be increased, but for now OpenMW only relies on a single clip plane
    constexpr unsigned int NumClipPlanes = 1;

    void setClipPlane(osg::StateSet& stateset, unsigned int index, const osg::Vec4f& plane,
        osg::StateAttribute::OverrideValue value = osg::StateAttribute::ON);

    void updateClipPlane(osg::StateSet& stateset, unsigned int index, const osg::Vec4f& plane);

    void setClipPlaneMode(osg::StateSet& stateset, unsigned int index,
        osg::StateAttribute::OverrideValue value = osg::StateAttribute::ON);
}

#endif