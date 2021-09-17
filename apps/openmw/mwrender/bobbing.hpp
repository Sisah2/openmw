#ifndef OPENMW_MWRENDER_BOBBING_HPP
#define OPENMW_MWRENDER_BOBBING_HPP 1

#include <cmath>

#include <osg/Vec3d>

namespace MWRender
{

struct BobbingInfo
{
    bool mHandBobEnabled;

    float mSneakOffset;
    bool mLandingShake;
    float mLandingOffset;

    float mInertiaPitch;
    float mInertiaYaw;
};

} // namespace MWRender

#endif // OPENMW_MWRENDER_BOBBING_HPP
