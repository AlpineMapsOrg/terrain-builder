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

std::array<ctb::TileCoordinate, 4> TopDownTiler::subtiles(const ctb::TileCoordinate& tile_id) const
{
    return {
        ctb::TileCoordinate{tile_id.zoom + 1, ctb::TilePoint(tile_id) * 2u + glm::uvec2(0, scheme() != Tile::Scheme::Tms)},
        ctb::TileCoordinate{tile_id.zoom + 1, ctb::TilePoint(tile_id) * 2u + glm::uvec2(1, scheme() != Tile::Scheme::Tms)},
        ctb::TileCoordinate{tile_id.zoom + 1, ctb::TilePoint(tile_id) * 2u + glm::uvec2(0, scheme() == Tile::Scheme::Tms)},
        ctb::TileCoordinate{tile_id.zoom + 1, ctb::TilePoint(tile_id) * 2u + glm::uvec2(1, scheme() == Tile::Scheme::Tms)}};
}

std::vector<Tile> TopDownTiler::generateTiles(ctb::i_zoom zoom_level, ctb::TilePoint tilepoint) const
{
    const auto tile_ids = subtiles({ zoom_level, tilepoint });
    std::vector<Tile> tiles;
    for (const auto& tile_id : tile_ids) {
        Tile t;
        t.zoom = tile_id.zoom;
        t.point = tile_id;
        const auto tms_y = (scheme() == Tile::Scheme::Tms) ? t.point.y : n_y_tiles(t.zoom) - t.point.y - 1;
        t.tileSize = tile_size();
        t.gridSize = grid_size();
        ctb::CRSBounds srs_bounds = grid().srsBounds(ctb::TileCoordinate(tile_id.zoom, tile_id.x, tms_y), border_south_east() == Tile::Border::Yes);
        t.srsBounds = srs_bounds;
        t.srs_epsg = grid().getEpsgCode();
        if (bounds().overlaps(srs_bounds))
            tiles.push_back(std::move(t));
    }

    return tiles;
}
