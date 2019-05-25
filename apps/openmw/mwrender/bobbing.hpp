#ifndef OPENMW_MWRENDER_BOBBING_HPP
#define OPENMW_MWRENDER_BOBBING_HPP 1

#include <cmath>

#include <osg/Vec3d>

namespace MWRender
{

struct BobbingInfo
{
    bool mHeadBobEnabled;
    bool mHandBobEnabled;
    float mCycle; /// Between 0.f and 2 * PI
    float mSpeedSmoothed; /// Smoothed movement speed
    float mAnimSpeed;

    /// Returned y-axis value is the roll about y-axis NOT y-offset
    void getOffsets(osg::Vec3d& outHeadBobbing, osg::Vec3d& outHandBobbing);
};

} // namespace MWRender

#endif // OPENMW_MWRENDER_BOBBING_HPP
