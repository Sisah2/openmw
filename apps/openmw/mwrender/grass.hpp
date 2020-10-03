#ifndef GAME_MWRENDER_GRASS_H
#define GAME_MWRENDER_GRASS_H

#include <string>

#include <osg/Group>

#include <components/esm/defs.hpp>

#include <components/sceneutil/statesetupdater.hpp>

namespace MWRender
{
    bool isGrassItem(const std::string& model);
}

#endif
