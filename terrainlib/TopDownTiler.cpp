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

TopDownTiler::TopDownTiler(const ctb::Grid& grid, const ctb::CRSBounds& bounds, Tile::Border border, Tile::Scheme scheme)
    : Tiler(grid, bounds, border, scheme)
{
}

std::array<Tile::Id, 4> TopDownTiler::subtiles(const Tile::Id& tile_id) const
{
    return {
        Tile::Id { tile_id.zoom_level + 1, tile_id.coords * 2u + glm::uvec2(0, tile_id.scheme != Tile::Scheme::Tms), tile_id.scheme },
        Tile::Id { tile_id.zoom_level + 1, tile_id.coords * 2u + glm::uvec2(1, tile_id.scheme != Tile::Scheme::Tms), tile_id.scheme },
        Tile::Id { tile_id.zoom_level + 1, tile_id.coords * 2u + glm::uvec2(0, tile_id.scheme == Tile::Scheme::Tms), tile_id.scheme },
        Tile::Id { tile_id.zoom_level + 1, tile_id.coords * 2u + glm::uvec2(1, tile_id.scheme == Tile::Scheme::Tms), tile_id.scheme }
    };
}

std::vector<Tile> TopDownTiler::generateTiles(const Tile::Id& parent_id) const
{
    assert(parent_id.scheme == scheme());
    const auto tile_ids = subtiles(parent_id.to(scheme()));
    std::vector<Tile> tiles;
    for (const auto& tile_id : tile_ids) {
        Tile t;
//        t.id = { tile_id.zoom, tile_id, scheme() };
        t.zoom = tile_id.zoom_level;
        t.point = tile_id.coords;
        t.tileSize = tile_size();
        t.gridSize = grid_size();
        ctb::CRSBounds srs_bounds = grid().srsBounds(tile_id, border_south_east() == Tile::Border::Yes);
        t.srsBounds = srs_bounds;
        t.srs_epsg = grid().getEpsgCode();
        if (bounds().overlaps(srs_bounds))
            tiles.push_back(std::move(t));
    }

    return tiles;
}
