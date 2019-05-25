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

void BobbingInfo::getOffsets(osg::Vec3d& outHeadOffset, osg::Vec3d& outHandOffset)
{
    static const float maximum = 2.f;
    static const float softLimit = 256.f; // Beyond this speed, head bobbing gradually becomes less pronounced
    static const float headRoll = clamp(Settings::Manager::getFloat("head bobbing roll", "Camera"), -maximum, maximum);
    static const float headSway = clamp(Settings::Manager::getFloat("head bobbing lateral sway", "Camera"), -maximum, maximum);
    static const float headBounce = clamp(Settings::Manager::getFloat("head bobbing vertical bounce", "Camera"), -maximum, maximum);

    float level = std::sin(mCycle);

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

    outHeadOffset.x() = level * scalar * headSway * 1.5f;
    outHeadOffset.y() = level * scalar * headRoll * 0.001f;
    outHeadOffset.z() = scalar * headBounce * 2.5f;
    outHeadOffset.z() -= std::abs(level) * scalar * headBounce * 5.f;

    static const float handSway = clamp(Settings::Manager::getFloat("hand bobbing lateral sway", "Camera"), -maximum, maximum);
    static const float handBounce = clamp(Settings::Manager::getFloat("hand bobbing vertical bounce", "Camera"), -maximum, maximum);

    outHandOffset.x() = scalar * handBounce * 0.007f;
    outHandOffset.x() -= std::abs(level) * scalar * handBounce * 0.015f;
    outHandOffset.z() = level * scalar * handSway * 0.015f;

    static const float handInertia = clamp(Settings::Manager::getFloat("hand inertia", "Camera"), -maximum, maximum);

    outHandOffset.x() += mInertiaPitch * scalar * handInertia * 0.08f;
    outHandOffset.z() += mInertiaYaw * scalar * handInertia * 0.08f;
}

} // namespace MWRender
