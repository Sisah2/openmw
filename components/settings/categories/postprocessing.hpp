#ifndef OPENMW_COMPONENTS_SETTINGS_CATEGORIES_POSTPROCESSING_H
#define OPENMW_COMPONENTS_SETTINGS_CATEGORIES_POSTPROCESSING_H

#include <components/settings/sanitizerimpl.hpp>
#include <components/settings/settingvalue.hpp>

#include <osg/Math>
#include <osg/Vec2f>
#include <osg/Vec3f>

#include <cstdint>
#include <string>
#include <string_view>

namespace Settings
{
    struct PostProcessingCategory : WithIndex
    {
        using WithIndex::WithIndex;

        SettingValue<bool> mEnabled{ mIndex, "Post Processing", "enabled" };
        SettingValue<std::vector<std::string>> mChain{ mIndex, "Post Processing", "chain" };
        SettingValue<float> mAutoExposureSpeed{ mIndex, "Post Processing", "auto exposure speed",
            makeMaxStrictSanitizerFloat(0.0001f) };
        SettingValue<bool> mTransparentPostpass{ mIndex, "Post Processing", "transparent postpass" };
        SettingValue<int> mNormalsFallbackRenderingDistance{ mIndex, "Post Processing", "normals fallback view distance" };

        SettingValue<int> mTest1{ mIndex, "Post Processing", "test1" };
        SettingValue<int> mTest2{ mIndex, "Post Processing", "test2" };
        SettingValue<int> mTest3{ mIndex, "Post Processing", "test3" };
        SettingValue<int> mTest4{ mIndex, "Post Processing", "test4" };
    };
}

#endif
