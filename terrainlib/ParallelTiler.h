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

#include "Tiler.h"

class ParallelTiler : public Tiler {
public:
    ParallelTiler(const ctb::Grid& grid, const ctb::CRSBounds& bounds, Tile::Border border, Tile::Scheme scheme);

    [[nodiscard]] std::vector<Tile> generateTiles(ctb::i_zoom zoom_level) const;
    [[nodiscard]] std::vector<Tile> generateTiles(const std::pair<ctb::i_zoom, ctb::i_zoom>& zoom_range) const;

    [[nodiscard]] ctb::TileCoordinate southWestTile(ctb::i_zoom zoom_level) const;
    [[nodiscard]] ctb::TileCoordinate northEastTile(ctb::i_zoom zoom_level) const;
};
