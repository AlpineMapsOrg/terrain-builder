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

#include "TopDownTiler.h"

TopDownTiler::TopDownTiler(const ctb::Grid& grid, const ctb::CRSBounds& bounds, tile::Border border, tile::Scheme scheme)
    : Tiler(grid, bounds, border, scheme)
{
}

std::vector<tile::Descriptor> TopDownTiler::generateTiles(const tile::Id& parent_id) const
{
    assert(parent_id.scheme == scheme());
    const auto tile_ids = parent_id.to(scheme()).children();
    std::vector<tile::Descriptor> tiles;
    for (const auto& tile_id : tile_ids) {
        tile::Descriptor t = tile_for(tile_id);
        if (intersect(bounds(), t.srsBounds))
            tiles.push_back(std::move(t));
    }

    return tiles;
}
