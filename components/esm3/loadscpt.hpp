#ifndef OPENMW_ESM_SCPT_H
#define OPENMW_ESM_SCPT_H

#include <cstdint>
#include <string>
#include <vector>

#include <components/esm/defs.hpp>
#include <components/esm/refid.hpp>

namespace ESM
{

    class ESMReader;
    class ESMWriter;

    /*
     * Script definitions
     */

    class Script
    {
    public:
        constexpr static RecNameInts sRecordId = REC_SCPT;

        /// Return a string descriptor for this record type. Currently used for debugging / error logs only.
        static std::string_view getRecordType() { return "Script"; }

        uint32_t mRecordFlags;
        RefId mId;

        std::uint32_t mNumShorts;
        std::uint32_t mNumLongs;
        std::uint32_t mNumFloats;

        /// Variable names generated by script-precompiling in the editor.
        /// \warning Do not use this field. OpenCS currently does not precompile scripts.
        std::vector<std::string> mVarNames;

        /// Bytecode generated from script-precompiling in the editor.
        /// \warning Do not use this field. OpenCS currently does not precompile scripts.
        std::vector<unsigned char> mScriptData;

        /// Script source code
        std::string mScriptText;

        void load(ESMReader& esm, bool& isDeleted);
        void save(ESMWriter& esm, bool isDeleted = false) const;

        void blank();
        ///< Set record to default state (does not touch the ID/index).
    };

    std::uint32_t computeScriptStringTableSize(const std::vector<std::string>& varNames);
}
#endif
