#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_SERIALIZATION_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_SERIALIZATION_H

#include <cstddef>
#include <cstdint>
#include <vector>

struct rcConfig;

namespace DetourNavigator
{
    class RecastMesh;
    struct PreparedNavMeshData;
    struct Settings;

    constexpr char recastMeshMagic[] = {'r', 'c', 's', 't'};
    constexpr std::uint32_t recastMeshVersion = 1;

    constexpr char preparedNavMeshDataMagic[] = {'p', 'n', 'a', 'v'};
    constexpr std::uint32_t preparedNavMeshDataVersion = 1;

    std::vector<std::byte> serialize(float recastScaleFactor, const rcConfig& config, const RecastMesh& value);

    std::vector<std::byte> serialize(const PreparedNavMeshData& value);

    bool deserialize(const std::vector<std::byte>& data, PreparedNavMeshData& value);
}

#endif
