#ifndef OPENMW_TEST_SUITE_DETOURNAVIGATOR_SETTINGS_H
#define OPENMW_TEST_SUITE_DETOURNAVIGATOR_SETTINGS_H

#include <components/detournavigator/settings.hpp>

namespace DetourNavigator
{
    namespace Tests
    {
        inline Settings makeSettings()
        {
            Settings result;
            result.mEnableWriteRecastMeshToFile = false;
            result.mEnableWriteNavMeshToFile = false;
            result.mEnableRecastMeshFileNameRevision = false;
            result.mEnableNavMeshFileNameRevision = false;
            result.mBorderSize = 16;
            result.mCellHeight = 0.2f;
            result.mCellSize = 0.2f;
            result.mDetailSampleDist = 6;
            result.mDetailSampleMaxError = 1;
            result.mMaxClimb = 34;
            result.mMaxSimplificationError = 1.3f;
            result.mMaxSlope = 49;
            result.mRecastScaleFactor = 0.017647058823529415f;
            result.mSwimHeightScale = 0.89999997615814208984375f;
            result.mMaxEdgeLen = 12;
            result.mMaxNavMeshQueryNodes = 2048;
            result.mMaxVertsPerPoly = 6;
            result.mRegionMergeSize = 20;
            result.mRegionMinSize = 8;
            result.mTileSize = 64;
            result.mWaitUntilMinDistanceToPlayer = std::numeric_limits<int>::max();
            result.mAsyncNavMeshUpdaterThreads = 1;
            result.mMaxNavMeshTilesCacheSize = 1024 * 1024;
            result.mMaxPolygonPathSize = 1024;
            result.mMaxSmoothPathSize = 1024;
            result.mMaxPolys = 4096;
            result.mMaxTilesNumber = 512;
            result.mMinUpdateInterval = std::chrono::milliseconds(50);
            result.mWriteToNavMeshDb = true;
            return result;
        }
    }
}

#endif
