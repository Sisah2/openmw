#include "serialization.hpp"

#include "preparednavmeshdata.hpp"
#include "recastmesh.hpp"
#include "recast.hpp"

#include <components/misc/endianness.hpp>

#include <Recast.h>
#include <RecastAlloc.h>

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace DetourNavigator
{
namespace
{
    struct SizeAccumulator
    {
        std::size_t mValue = 0;

        template <class T>
        auto operator()(const T& /*value*/)
            -> std::enable_if_t<std::is_arithmetic_v<T>>
        {
            mValue += sizeof(T);
        }

        template <class T>
        inline auto operator()(const T& /*value*/)
            -> std::enable_if_t<!std::is_arithmetic_v<T>>;

        template <class T>
        auto operator()(const T* /*data*/, std::size_t count)
            -> std::enable_if_t<std::is_arithmetic_v<T>>
        {
            mValue += count * sizeof(T);
        }
    };

    struct BinaryWriter
    {
        std::byte* mDest;
        const std::byte* const mEnd;

        template <class T>
        auto operator()(T value)
            -> std::enable_if_t<std::is_arithmetic_v<T>>
        {
            if (mEnd - mDest < static_cast<std::ptrdiff_t>(sizeof(value)))
                throw std::runtime_error("Not enough space");
            value = Misc::toLittleEndian(value);
            std::memcpy(mDest, &value, sizeof(value));
            mDest += sizeof(value);
        }

        template <class T>
        inline auto operator()(const T& value)
            -> std::enable_if_t<!std::is_arithmetic_v<T>>;

        template <class T>
        auto operator()(const T* data, std::size_t count)
            -> std::enable_if_t<std::is_arithmetic_v<T>>
        {
            const std::size_t size = sizeof(T) * count;
            if (mEnd - mDest < static_cast<std::ptrdiff_t>(size))
                throw std::runtime_error("Not enough space");
            if constexpr (Misc::IS_LITTLE_ENDIAN || sizeof(T) == 1)
            {
                std::memcpy(mDest, data, size);
                mDest += size;
            }
            else
            {
                std::for_each_n(data, data + count, std::ref(*this));
            }
        }
    };

    template <class T, class F>
    auto serialize(T* data, std::size_t size, F&& f)
        -> std::enable_if_t<std::is_arithmetic_v<T>>
    {
        f(data, size);
    }

    template <class T, std::size_t size, class F>
    auto serialize(T(& data)[size], F&& f)
        -> std::enable_if_t<std::is_arithmetic_v<T>>
    {
        f(data, size);
    }

    template <class F>
    void serialize(const osg::Vec2f& value, F&& f)
    {
        f(value.ptr(), 2);
    }

    template <class F>
    void serialize(const osg::Vec3f& value, F&& f)
    {
        f(value.ptr(), 3);
    }

    template <class F>
    void serialize(const Cell& value, F&& f)
    {
        f(value.mSize);
        f(value.mShift);
    }

    template <class T, class F>
    auto serialize(const std::vector<T>& value, F&& f)
        -> std::enable_if_t<std::is_arithmetic_v<T>>
    {
        f(value.size());
        f(value.data(), value.size());
    }

    template <class T, class F>
    auto serialize(const std::vector<T>& value, F&& f)
        -> std::enable_if_t<std::is_enum_v<T>>
    {
        f(value.size());
        f(reinterpret_cast<const std::underlying_type_t<T>*>(value.data()), value.size());
    }

    template <class T, class F>
    auto serialize(const std::vector<T>& values, F&& f)
        -> std::enable_if_t<!std::is_arithmetic_v<T> && !std::is_enum_v<T>>
    {
        f(values.size());
        std::for_each(values.begin(), values.end(), std::ref(f));
    }

    template <class F>
    void serialize(const rcConfig& value, F&& f)
    {
        f(value.width);
        f(value.height);
        f(value.tileSize);
        f(value.borderSize);
        f(value.cs);
        f(value.ch);
        f(value.bmin);
        f(value.bmax);
        f(value.walkableSlopeAngle);
        f(value.walkableHeight);
        f(value.walkableClimb);
        f(value.walkableRadius);
        f(value.maxEdgeLen);
        f(value.maxSimplificationError);
        f(value.minRegionArea);
        f(value.mergeRegionArea);
        f(value.maxVertsPerPoly);
        f(value.detailSampleDist);
        f(value.detailSampleMaxError);
    }

    template <class F>
    void serialize(const TileBounds& value, F&& f)
    {
        f(value.mMin);
        f(value.mMax);
    }

    template <class F>
    void serialize(const Mesh& value, F&& f)
    {
        f(value.getIndices());
        f(value.getVertices());
        f(value.getAreaTypes());
    }

    template <class F>
    void serialize(const Heightfield& value, F&& f)
    {
        f(value.mBounds);
        f(value.mLength);
        f(value.mMinHeight);
        f(value.mMaxHeight);
        f(value.mShift);
        f(value.mScale);
        f(value.mHeights);
    }

    template <class F>
    void serialize(const FlatHeightfield& value, F&& f)
    {
        f(value.mBounds);
        f(value.mHeight);
    }

    template <class F>
    void serialize(const RecastMesh& value, F&& f)
    {
        f(value.getMesh());
        f(value.getWater());
        f(value.getHeightfields());
        f(value.getFlatHeightfields());
    }

    template <class F>
    void serialize(float recastScaleFactor, const rcConfig& config, const RecastMesh& recastMesh, F&& f)
    {
        f(DetourNavigator::recastMeshMagic);
        f(DetourNavigator::recastMeshVersion);
        f(recastScaleFactor);
        f(config);
        f(recastMesh);
    }

    template <class T, class F>
    auto serialize(T& value, F&& f)
        -> std::enable_if_t<std::is_same_v<std::decay_t<T>, rcPolyMesh>>
    {
        f(value.nverts);
        f(value.npolys);
        f(value.maxpolys);
        f(value.nvp);
        f(value.bmin);
        f(value.bmax);
        f(value.cs);
        f(value.ch);
        f(value.borderSize);
        f(value.maxEdgeError);
        f(value.verts, getVertsLength(value));
        f(value.polys, getPolysLength(value));
        f(value.regs, getRegsLength(value));
        f(value.flags, getFlagsLength(value));
        f(value.areas, getAreasLength(value));
    }

    template <class T, class F>
    auto serialize(T& value, F&& f)
        -> std::enable_if_t<std::is_same_v<std::decay_t<T>, rcPolyMeshDetail>>
    {
        f(value.nmeshes);
        f(value.meshes, getMeshesLength(value));
        f(value.nverts);
        f(value.verts, getVertsLength(value));
        f(value.ntris);
        f(value.tris, getTrisLength(value));
    }

    template <class F>
    auto serialize(const PreparedNavMeshData& value, F&& f)
    {
        f(DetourNavigator::preparedNavMeshDataMagic);
        f(DetourNavigator::preparedNavMeshDataVersion);
        f(value.mUserId);
        f(value.mCellSize);
        f(value.mCellHeight);
        f(value.mPolyMesh);
        f(value.mPolyMeshDetail);
    }

    template <class T>
    inline auto SizeAccumulator::operator()(const T& value)
        -> std::enable_if_t<!std::is_arithmetic_v<T>>
    {
        serialize(value, *this);
    }

    template <class T>
    inline auto BinaryWriter::operator()(const T& value)
        -> std::enable_if_t<!std::is_arithmetic_v<T>>
    {
        serialize(value, *this);
    }
}
} // namespace DetourNavigator

namespace DetourNavigator
{
    std::vector<std::byte> serialize(float recastScaleFactor, const rcConfig& config, const RecastMesh& recastMesh)
    {
        SizeAccumulator sizeAccumulator;
        serialize(recastScaleFactor, config, recastMesh, sizeAccumulator);
        std::vector<std::byte> result(sizeAccumulator.mValue);
        serialize(recastScaleFactor, config, recastMesh, BinaryWriter {result.data(), result.data() + result.size()});
        return result;
    }

    std::vector<std::byte> serialize(const PreparedNavMeshData& value)
    {
        SizeAccumulator sizeAccumulator;
        serialize(value, sizeAccumulator);
        std::vector<std::byte> result(sizeAccumulator.mValue);
        serialize(value, BinaryWriter {result.data(), result.data() + result.size()});
        return result;
    }
}
