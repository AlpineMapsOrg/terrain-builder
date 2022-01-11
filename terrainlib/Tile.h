/*****************************************************************************
 * Alpine Terrain Builder
 * Copyright (C) 2022 alpinemaps.org
 * Copyright (C) 2022 Adam Celarek <family name at cg tuwien ac at>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#ifndef TILE_H
#define TILE_H

#include <glm/glm.hpp>
#include "ctb/types.hpp"
#include "ctb/TileCoordinate.hpp"

struct Tile
{
  ctb::TilePoint point;  // int / used to generate file name
  ctb::i_zoom zoom;
  ctb::CRSBounds srsBounds;

  // some tiling schemes require a border (e.g. cesium heightmap https://github.com/CesiumGS/cesium/wiki/heightmap-1%2E0).
  // grid bounds does not contain that border (e.g. 64 width)
  // tile bounds contains that border (e.g. 65 width)
  ctb::i_tile gridSize;
  ctb::i_tile tileSize;
};

#endif // TILE_H
