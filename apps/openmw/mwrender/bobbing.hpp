#ifndef OPENMW_MWRENDER_BOBBING_HPP
#define OPENMW_MWRENDER_BOBBING_HPP 1

#include <cmath>

#include <osg/Vec3d>

namespace MWRender
{

struct BobbingInfo
{
    bool mHandBobEnabled;

    float mSpeedSmoothed; /// Smoothed movement speed
    float mAnimSpeed;
    float mSneakOffset;
    bool mLandingShake;
    float mLandingOffset;

    float mInertiaPitch;
    float mInertiaYaw;

    /// Returned y-axis value for head bobbing is the roll about y-axis NOT y-offset,
    void getOffsets(osg::Vec3d& outHandBobbing);
};

} // namespace MWRender

#endif // OPENMW_MWRENDER_BOBBING_HPP
