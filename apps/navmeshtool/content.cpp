#include "content.hpp"

#include <components/debug/debuglog.hpp>
#include <components/esm/cellref.hpp>
#include <components/esm/defs.hpp>
#include <components/esm/esmreader.hpp>
#include <components/esm/loadacti.hpp>
#include <components/esm/loadcell.hpp>
#include <components/esm/loadcont.hpp>
#include <components/esm/loaddoor.hpp>
#include <components/esm/loadgmst.hpp>
#include <components/esm/loadland.hpp>
#include <components/esm/loadstat.hpp>
#include <components/esm/variant.hpp>
#include <components/files/collections.hpp>
#include <components/files/multidircollection.hpp>
#include <components/misc/resourcehelpers.hpp>
#include <components/misc/stringops.hpp>

#include <osg/Vec2i>

#include <algorithm>
#include <cstddef>
#include <map>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace NavMeshTool
{
    namespace
    {
        template <class T>
        struct Record
        {
            bool mDeleted;
            T mValue;

            template <class ... Args>
            Record(bool deleted, Args&& ... args) : mDeleted(deleted), mValue(std::forward<Args>(args) ...) {}
        };

        struct GetKey
        {
            template <class T>
            decltype(auto) operator()(const T& v) const
            {
                return (v.mId);
            }

            const ESM::CellId& operator()(const ESM::Cell& v) const
            {
                return v.mCellId;
            }

            osg::Vec2i operator()(const ESM::Land& v) const
            {
                return osg::Vec2i(v.mX, v.mY);
            }

            const ESM::RefNum& operator()(const CellRef& v) const
            {
                return v.mRefNum;
            }

            template <class T>
            decltype(auto) operator()(const Record<T>& v) const
            {
                return (*this)(v.mValue);
            }
        };

        template <class T>
        using Records = std::vector<Record<T>>;

        struct CellRecords
        {
            Records<ESM::Cell> mValues;
            std::map<std::string, std::size_t> mByName;
            std::map<osg::Vec2i, std::size_t> mByPosition;
        };

        template <class T, class = std::void_t<>>
        struct HasId : std::false_type {};

        template <class T>
        struct HasId<T, std::void_t<decltype(T::mId)>> : std::true_type {};

        template <class T>
        constexpr bool hasId = HasId<T>::value;

        template <class T>
        auto loadRecord(ESM::ESMReader& reader, Records<T>& records)
            -> std::enable_if_t<hasId<T>>
        {
            T record;
            bool deleted = false;
            record.load(reader, deleted);
            Misc::StringUtils::lowerCaseInPlace(record.mId);
            if (Misc::ResourceHelpers::isHiddenMarker(record.mId))
                return;
            records.emplace_back(deleted, std::move(record));
        }

        template <class T>
        auto loadRecord(ESM::ESMReader& reader, Records<T>& records)
            -> std::enable_if_t<!hasId<T>>
        {
            T record;
            bool deleted = false;
            record.load(reader, deleted);
            records.emplace_back(deleted, std::move(record));
        }

        void loadRecord(ESM::ESMReader& reader, CellRecords& records)
        {
            ESM::Cell record;
            bool deleted = false;
            record.loadNameAndData(reader, deleted);
            Misc::StringUtils::lowerCaseInPlace(record.mName);

            if ((record.mData.mFlags & ESM::Cell::Interior) != 0)
            {
                const auto it = records.mByName.find(record.mName);
                if (it == records.mByName.end())
                {
                    record.loadCell(reader, true);
                    records.mByName.emplace_hint(it, record.mName, records.mValues.size());
                    records.mValues.emplace_back(deleted, std::move(record));
                }
                else
                {
                    Record<ESM::Cell>& old = records.mValues[it->second];
                    old.mValue.mData = record.mData;
                    old.mValue.loadCell(reader, true);
                }
            }
            else
            {
                const osg::Vec2i position(record.mData.mX, record.mData.mY);
                const auto it = records.mByPosition.find(position);
                if (it == records.mByPosition.end())
                {
                    record.loadCell(reader, true);
                    records.mByPosition.emplace_hint(it, position, records.mValues.size());
                    records.mValues.emplace_back(deleted, std::move(record));
                }
                else
                {
                    Record<ESM::Cell>& old = records.mValues[it->second];
                    old.mValue.mData = record.mData;
                    old.mValue.loadCell(reader, true);
                }
            }
        }

        struct ShallowContent
        {
            Records<ESM::Activator> mActivators;
            CellRecords mCells;
            Records<ESM::Container> mContainers;
            Records<ESM::Door> mDoors;
            Records<ESM::GameSetting> mGameSettings;
            Records<ESM::Land> mLands;
            Records<ESM::Static> mStatics;
        };

        void loadRecord(const ESM::NAME& name, ESM::ESMReader& reader, ShallowContent& content)
        {
            switch (name.toInt())
            {
                case ESM::REC_ACTI: return loadRecord(reader, content.mActivators);
                case ESM::REC_CELL: return loadRecord(reader, content.mCells);
                case ESM::REC_CONT: return loadRecord(reader, content.mContainers);
                case ESM::REC_DOOR: return loadRecord(reader, content.mDoors);
                case ESM::REC_GMST: return loadRecord(reader, content.mGameSettings);
                case ESM::REC_LAND: return loadRecord(reader, content.mLands);
                case ESM::REC_STAT: return loadRecord(reader, content.mStatics);
            }

            reader.skipRecord();
        }

        ESM::ESMReader loadEsm(ESM::ESMReader& reader, ShallowContent& content)
        {
            Log(Debug::Info) << "Loading content file " << reader.getName();

            while (reader.hasMoreRecs())
            {
                const ESM::NAME recName = reader.getRecName();
                reader.getRecHeader();
                loadRecord(recName, reader, content);
            }

            return reader;
        }

        ShallowContent shallowLoad(const std::vector<std::string>& contentFiles, const Files::Collections& fileCollections,
            std::vector<ESM::ESMReader>& readers, ToUTF8::Utf8Encoder& encoder)
        {
            ShallowContent result;

            for (std::size_t i = 0; i < contentFiles.size(); ++i)
            {
                const std::string &file = contentFiles[i];
                const Files::MultiDirCollection& collection = fileCollections.getCollection(boost::filesystem::path(file).extension().string());

                ESM::ESMReader& reader = readers[i];
                reader.setEncoder(&encoder);
                reader.setIndex(static_cast<int>(i));
                reader.setGlobalReaderList(&readers);
                reader.open(collection.getPath(file).string());

                loadEsm(readers[i], result);
            }

            return result;
        }

        struct LessById
        {
            template <class T>
            bool operator()(const T& lhs, const T& rhs) const
            {
                return lhs.mId < rhs.mId;
            }

            template <class T>
            bool operator()(const T& lhs, std::string_view rhs) const
            {
                return lhs.mId < rhs;
            }
        };

        template <class T>
        std::vector<T> prepareRecords(Records<T>& records)
        {
            GetKey getKey;
            const auto lessByKey = [&] (const auto& l, const auto& r) { return getKey(l) < getKey(r); };
            const auto equalByKey = [&] (const auto& l, const auto& r) { return getKey(l) == getKey(r); };
            std::stable_sort(records.begin(), records.end(), std::not_fn(lessByKey));
            records.erase(std::unique(records.begin(), records.end(), equalByKey), records.end());
            std::reverse(records.begin(), records.end());
            std::vector<T> result;
            for (Record<T>& v : records)
                if (!v.mDeleted)
                    result.emplace_back(std::move(v.mValue));
            return result;
        }

        std::vector<ESM::Cell> prepareRecords(Records<ESM::Cell>& records)
        {
            std::vector<ESM::Cell> result;
            for (Record<ESM::Cell>& v : records)
                if (!v.mDeleted)
                    result.emplace_back(std::move(v.mValue));
            return result;
        }

        struct WithType
        {
            ESM::RecNameInts mType;

            template <class T>
            RefIdWithType operator()(const T& v) const { return {v.mId, mType}; }
        };

        template <class T>
        void addRefIdsTypes(const std::vector<T>& values, std::vector<RefIdWithType>& refIdsTypes)
        {
            std::transform(values.begin(), values.end(), std::back_inserter(refIdsTypes),
                           WithType {static_cast<ESM::RecNameInts>(T::sRecordId)});
        }

        void addRefIdsTypes(Content& content)
        {
            content.mRefIdTypes.reserve(
                content.mActivators.size()
                + content.mContainers.size()
                + content.mDoors.size()
                + content.mStatics.size()
            );

            addRefIdsTypes(content.mActivators, content.mRefIdTypes);
            addRefIdsTypes(content.mContainers, content.mRefIdTypes);
            addRefIdsTypes(content.mDoors, content.mRefIdTypes);
            addRefIdsTypes(content.mStatics, content.mRefIdTypes);

            std::sort(content.mRefIdTypes.begin(), content.mRefIdTypes.end(), LessById {});
        }

        template <class F>
        auto returnAs(F&& f)
        {
            using Result = decltype(std::forward<F>(f)(ESM::Static {}));
            if constexpr (!std::is_same_v<Result, void>)
                return Result {};
        }

        template <class T, class F>
        auto withStatic(std::string_view refId, const std::vector<T>& values, F&& f)
        {
            const auto it = std::lower_bound(values.begin(), values.end(), refId, LessById {});

            if (it == values.end() && it->mId != refId)
                return returnAs(std::forward<F>(f));

            return std::forward<F>(f)(*it);
        }

        template <class F>
        auto withStatic(std::string_view refId, ESM::RecNameInts type, const Content& content, F&& f)
        {
            switch (type)
            {
                case ESM::REC_ACTI: return withStatic(refId, content.mActivators, std::forward<F>(f));
                case ESM::REC_CONT: return withStatic(refId, content.mContainers, std::forward<F>(f));
                case ESM::REC_DOOR: return withStatic(refId, content.mDoors, std::forward<F>(f));
                case ESM::REC_STAT: return withStatic(refId, content.mStatics, std::forward<F>(f));
                default: break;
            }

            return returnAs(std::forward<F>(f));
        }

        ESM::RecNameInts getType(const Content& content, std::string_view refId)
        {
            const auto it = std::lower_bound(content.mRefIdTypes.begin(), content.mRefIdTypes.end(), refId, LessById {});
            if (it == content.mRefIdTypes.end() || it->mId != refId)
                return {};
            return it->mType;
        }
    }

    Content::~Content() {}

    Content loadContent(const std::vector<std::string>& contentFiles, const Files::Collections& fileCollections,
        std::vector<ESM::ESMReader>& readers, ToUTF8::Utf8Encoder& encoder)
    {
        Log(Debug::Info) << "Loading content files...";

        ShallowContent content = shallowLoad(contentFiles, fileCollections, readers, encoder);

        Log(Debug::Info) << "Loaded "
            << content.mActivators.size() << " activators, "
            << content.mCells.mValues.size() << " cells, "
            << content.mContainers.size() << " containers, "
            << content.mDoors.size() << " doors, "
            << content.mGameSettings.size() << " game settings, "
            << content.mLands.size() << " lands, "
            << content.mStatics.size() << " statics";

        Content result;

        result.mActivators = prepareRecords(content.mActivators);
        result.mCells = prepareRecords(content.mCells.mValues);
        result.mContainers = prepareRecords(content.mContainers);
        result.mDoors = prepareRecords(content.mDoors);
        result.mGameSettings = prepareRecords(content.mGameSettings);
        result.mLands = prepareRecords(content.mLands);
        result.mStatics = prepareRecords(content.mStatics);

        addRefIdsTypes(result);

        Log(Debug::Info) << "Prepared "
            << result.mActivators.size() << " unique activators, "
            << result.mCells.size() << " unique cells, "
            << result.mContainers.size() << " unique containers, "
            << result.mDoors.size() << " unique doors, "
            << result.mGameSettings.size() << " unique game settings, "
            << result.mLands.size() << " unique lands, "
            << result.mStatics.size() << " unique statics";

        return result;
    }

    std::string_view getModel(const Content& content, std::string_view refId, ESM::RecNameInts type)
    {
        return withStatic(refId, type, content, [] (const auto& v) { return std::string_view(v.mModel); });
    }

    ESM::Variant getGameSetting(const std::vector<ESM::GameSetting>& records, const std::string& id)
    {
        const std::string lower = Misc::StringUtils::lowerCase(id);
        auto it = std::lower_bound(records.begin(), records.end(), lower, LessById {});
        if (it == records.end() || it->mId != lower)
            throw std::runtime_error(std::string("Game settings \"") + id + "\" is not found");
        return it->mValue;
    }

    std::vector<CellRef> loadCellRefs(const ESM::Cell& cell, const Content& content, std::vector<ESM::ESMReader>& readers)
    {
        std::vector<Record<CellRef>> cellRefs;

        for (std::size_t i = 0; i < cell.mContextList.size(); i++)
        {
            ESM::ESMReader& reader = readers[static_cast<std::size_t>(cell.mContextList[i].index)];
            cell.restore(reader, static_cast<int>(i));
            ESM::CellRef cellRef;
            bool deleted = false;
            while (ESM::Cell::getNextRef(reader, cellRef, deleted))
            {
                Misc::StringUtils::lowerCaseInPlace(cellRef.mRefID);
                const ESM::RecNameInts type = getType(content, cellRef.mRefID);
                if (type == ESM::RecNameInts {})
                    continue;
                cellRefs.emplace_back(deleted, type, cellRef.mRefNum, std::move(cellRef.mRefID),
                                      cellRef.mScale, cellRef.mPos);
            }
        }

        Log(Debug::Debug) << "Loaded " << cellRefs.size() << " cell refs";

        return prepareRecords(cellRefs);
    }
}
