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

#pragma once

#include "ctb/TileCoordinate.hpp"
#include "ctb/types.hpp"
#include <glm/glm.hpp>

struct Tile {
    enum class Border {
        Yes = 1,
        No = 0
    };

    // The difference between TMS and slippyMap is whether y starts counting from the bottom (south) or top (north).
    // https://www.maptiler.com/google-maps-coordinates-tile-bounds-projection/#1/-16.88/79.02
    //
    enum class Scheme {
        Tms, // southern most tile is y = 0
        SlippyMap // aka Google, XYZ, webmap tiles; northern most tile is y = 0
    };

    ctb::TilePoint point; // int / used to generate file name
    ctb::i_zoom zoom;
    ctb::CRSBounds srsBounds;
    int srs_epsg;

    // some tiling schemes require a border (e.g. cesium heightmap https://github.com/CesiumGS/cesium/wiki/heightmap-1%2E0).
    // grid bounds does not contain that border (e.g. 64 width)
    // tile bounds contains that border (e.g. 65 width)
    ctb::i_tile gridSize;
    ctb::i_tile tileSize;
};
