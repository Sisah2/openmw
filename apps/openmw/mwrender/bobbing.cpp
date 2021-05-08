#include <components/settings/settings.hpp>

#include "bobbing.hpp"

namespace
{
    float clamp(float val, float min, float max)
    {
        return std::min(max, std::max(min, val));
    }
}

namespace MWRender
{

void BobbingInfo::getOffsets(osg::Vec3d& outHandOffset)
{
    static const float maximum = 3.f;
    static const float softLimit = 256.f;

    float scalar = std::min(mSpeedSmoothed / softLimit, 1.f);
    if (scalar < 1.f)
    {
        scalar = std::pow(scalar, 1/3); // Increase bob level at lower speeds
    }
    else if (mAnimSpeed > softLimit)
    {
        float asymptote = mAnimSpeed / softLimit;
        scalar = std::max(0.f, 1.f - asymptote / (1 + asymptote));
    }

    static const float handInertia = clamp(Settings::Manager::getFloat("hand inertia", "Camera"), -maximum, maximum);

    outHandOffset.x() += mInertiaPitch * scalar * handInertia * 0.08f;
    outHandOffset.z() += mInertiaYaw * scalar * handInertia * 0.08f;
}

} // namespace MWRender
