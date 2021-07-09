#include "generatenavmeshtile.hpp"

#include "makenavmesh.hpp"
#include "preparednavmeshdata.hpp"
#include "offmeshconnectionsmanager.hpp"
#include "settings.hpp"
#include "tilecachedrecastmeshmanager.hpp"
#include "serialization.hpp"

#include <components/debug/debuglog.hpp>

#include <osg/Vec3f>

#include <memory>
#include <stdexcept>
#include <vector>
#include <optional>
#include <functional>

namespace DetourNavigator
{
    GenerateNavMeshTile::GenerateNavMeshTile(std::string worldspace, const TilePosition& tilePosition,
            RecastMeshProvider recastMeshProvider, const osg::Vec3f& agentHalfExtents,
            const DetourNavigator::Settings& settings, std::weak_ptr<NavMeshTileConsumer> consumer)
        : mWorldspace(std::move(worldspace))
        , mTilePosition(tilePosition)
        , mRecastMeshProvider(recastMeshProvider)
        , mAgentHalfExtents(agentHalfExtents)
        , mSettings(settings)
        , mConsumer(std::move(consumer)) {}

    void GenerateNavMeshTile::doWork()
    {
        try
        {
            const auto consumer = mConsumer.lock();

            if (consumer == nullptr)
                return;

            const std::shared_ptr<RecastMesh> recastMesh = mRecastMeshProvider.getMesh(mWorldspace, mTilePosition);

            if (recastMesh == nullptr)
                return consumer->ignore();

            const Bounds bounds = getBounds(*recastMesh, mAgentHalfExtents, mSettings);

            if (isEmpty(bounds))
                return consumer->ignore();

            const rcConfig config = makeRecastConfig(mTilePosition, bounds, mAgentHalfExtents, mSettings);
            std::vector<std::byte> input = serialize(mSettings.mRecastScaleFactor, config, *recastMesh);
            const std::optional<NavMeshTileInfo> info = consumer->find(mWorldspace, mTilePosition, input);

            if (info.has_value() && info->mVersion == mSettings.mNavMeshVersion)
                return consumer->ignore();

            const auto data = prepareNavMeshTileData(config, *recastMesh, mAgentHalfExtents, mSettings);

            if (data == nullptr)
                return consumer->ignore();

            if (info.has_value())
                consumer->update(info->mTileId, mSettings.mNavMeshVersion, *data);
            else
                consumer->insert(mWorldspace, mTilePosition, mSettings.mNavMeshVersion, input, *data);
        }
        catch (const std::exception& e)
        {
            Log(Debug::Warning) << "Failed to generate navmesh for worldspace \"" << mWorldspace
                                << "\" tile " << mTilePosition << ": " << e.what();
        }
    }
}
