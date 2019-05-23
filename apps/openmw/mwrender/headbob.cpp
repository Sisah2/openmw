#include <components/settings/settings.hpp>

#include "headbob.hpp"

namespace MWRender
{

void HeadBobInfo::getHeadBobOffset(osg::Vec3d& result)
{
    static const float maximum = 2.f;
    static const float softLimit = 256.f;
    static const float roll = std::max(maximum, Settings::Manager::getFloat("head bobbing roll", "Camera"));
    static const float sway = std::max(maximum, Settings::Manager::getFloat("head bobbing lateral sway", "Camera"));
    static const float bounce = std::max(maximum, Settings::Manager::getFloat("head bobbing vertical bounce", "Camera"));

    float level = std::sin(mCycle);

    float scalar = std::min(mSpeedSmoothed / softLimit, 1.f);
    if (scalar == 1.f && mAnimSpeed > softLimit)
    {
        float asymptote = mAnimSpeed / softLimit;
        scalar = std::max(0.f, 1.f - asymptote / (1 + asymptote));
    }

    result.x() = level * scalar * sway * 1.5f;
    result.y() = scalar * bounce * 2.f;
    result.y() -= scalar * bounce * std::abs(level) * 4.f;
    result.z() = level * scalar * roll * 0.002f;
}

} // namespace MWRender
