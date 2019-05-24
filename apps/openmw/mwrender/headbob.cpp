#include <components/settings/settings.hpp>

#include "headbob.hpp"

namespace
{
    float clamp(float val, float min, float max)
    {
        return std::min(max, std::max(min, val));
    }
}

namespace MWRender
{

void HeadBobInfo::getHeadBobOffset(osg::Vec3d& result)
{
    static const float maximum = 2.f;
    static const float softLimit = 256.f; // Beyond this speed, head bobbing gradually becomes less pronounced
    static const float roll = clamp(Settings::Manager::getFloat("head bobbing roll", "Camera"), -maximum, maximum);
    static const float sway = clamp(Settings::Manager::getFloat("head bobbing lateral sway", "Camera"), -maximum, maximum);
    static const float bounce = clamp(Settings::Manager::getFloat("head bobbing vertical bounce", "Camera"), -maximum, maximum);

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

    result.x() = level * scalar * sway * 1.5f;
    result.y() = scalar * bounce * 2.5f;
    result.y() -= scalar * bounce * std::abs(level) * 5.f;
    result.z() = level * scalar * roll * 0.001f;

    if (!std::isfinite(result.length2()))
        result.set(0, 0, 0);
}

} // namespace MWRender
