#ifndef OPENMW_NAVMESHTOOL_CONTENT_H
#define OPENMW_NAVMESHTOOL_CONTENT_H

#include <components/esm/cellref.hpp>
#include <components/esm/defs.hpp>
#include <components/esm/variant.hpp>

#include <string_view>
#include <string>
#include <vector>

namespace ESM
{
    struct Activator;
    struct Cell;
    struct Container;
    struct Door;
    struct GameSetting;
    struct Land;
    struct Static;
}

namespace ToUTF8
{
    class Utf8Encoder;
}

namespace Files
{
    class Collections;
}

namespace NavMeshTool
{
    struct RefIdWithType
    {
        std::string_view mId;
        ESM::RecNameInts mType;
    };

    struct Content
    {
        std::vector<ESM::Activator> mActivators;
        std::vector<ESM::Cell> mCells;
        std::vector<ESM::Container> mContainers;
        std::vector<ESM::Door> mDoors;
        std::vector<ESM::GameSetting> mGameSettings;
        std::vector<ESM::Land> mLands;
        std::vector<ESM::Static> mStatics;
        std::vector<RefIdWithType> mRefIdTypes;

        Content() = default;
        Content(const Content&) = delete;
        Content(Content&&) = default;

        ~Content();
    };

    struct CellRef
    {
        ESM::RecNameInts mType;
        ESM::RefNum mRefNum;
        std::string mRefId;
        float mScale;
        ESM::Position mPos;

        CellRef(ESM::RecNameInts type, ESM::RefNum refNum, std::string&& refId, float scale, const ESM::Position& pos)
            : mType(type), mRefNum(refNum), mRefId(std::move(refId)), mScale(scale), mPos(pos) {}
    };

    Content loadContent(const std::vector<std::string>& contentFiles, const Files::Collections& fileCollections,
        std::vector<ESM::ESMReader>& readers, ToUTF8::Utf8Encoder& encoder);

    std::string_view getModel(const Content& content, std::string_view refId, ESM::RecNameInts type);

    ESM::Variant getGameSetting(const std::vector<ESM::GameSetting>& records, const std::string& id);

    std::vector<CellRef> loadCellRefs(const ESM::Cell& cell, const Content& content, std::vector<ESM::ESMReader>& readers);
}

#endif
