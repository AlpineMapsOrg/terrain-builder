/*****************************************************************************
 * Alpine Terrain Builder
 * Copyright (C) 2022 Adam Celarek
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
#include <array>
#include "Tile.h"
#include "Tiler.h"
#include "ctb/Grid.hpp"
#include "ctb/types.hpp"

class TopDownTiler : public Tiler {
public:
    TopDownTiler(const ctb::Grid& grid, const ctb::CRSBounds& bounds, Tile::Border border, Tile::Scheme scheme);

    std::array<ctb::TileCoordinate, 4> subtiles(const ctb::TileCoordinate& tile_id) const;
    std::vector<Tile> generateTiles(ctb::i_zoom zoom_level, ctb::TilePoint tilepoint) const;
};
