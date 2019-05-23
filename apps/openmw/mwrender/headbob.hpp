#ifndef OPENMW_MWRENDER_HEADBOB_H
#define OPENMW_MWRENDER_HEADBOB_H 1

#include <cmath>

#include <osg/Vec3d>

namespace MWRender
{

struct HeadBobInfo
{
    bool mEnabled;
    float mCycle; /// Between 0.f and 2 * PI
    float mSpeedSmoothed; /// Smoothed movement speed
    float mAnimSpeed;

    /// Returned z-axis value is roll NOT z-offset
    void getHeadBobOffset(osg::Vec3d& result);
};

} // namespace MWRender

#endif // OPENMW_MWRENDER_HEADBOB_H
