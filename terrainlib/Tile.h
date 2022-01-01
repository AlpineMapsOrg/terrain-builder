#ifndef TILE_H
#define TILE_H

#include <glm/glm.hpp>
#include "ctb/types.hpp"
#include "ctb/TileCoordinate.hpp"

struct Tile
{
  ctb::TilePoint point;  // int / used to generate file name
  unsigned zoom;
  ctb::CRSBounds srsBounds;

  // some tiling schemes require a border (e.g. cesium heightmap https://github.com/CesiumGS/cesium/wiki/heightmap-1%2E0).
  // grid bounds does not contain that border (e.g. 64 width)
  // tile bounds contains that border (e.g. 65 width)
  ctb::i_tile gridSize;
  ctb::i_tile tileSize;
};

#endif // TILE_H
