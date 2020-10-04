#include "grass.hpp"

#include <components/misc/stringops.hpp>

namespace MWRender
{
    bool isGrassItem(const std::string& model)
    {
        std::string mesh = Misc::StringUtils::lowerCase (model);
        if (mesh.find("grass\\") == 0)
            return true;

        return false;
    }
}
