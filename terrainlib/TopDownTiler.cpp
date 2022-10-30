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

struct TileId {
    unsigned zoom_level = unsigned(-1);
    glm::uvec2 coords;
    friend bool operator==(const TileId&, const TileId&) = default;
};

// from alpine terrain renderer (srs.cpp), and therefore already tested
std::array<TileId, 4> subtiles(const TileId& tile)
{
    return {
        TileId{tile.zoom_level + 1, tile.coords * 2u + glm::uvec2(0, 0)},
        TileId{tile.zoom_level + 1, tile.coords * 2u + glm::uvec2(1, 0)},
        TileId{tile.zoom_level + 1, tile.coords * 2u + glm::uvec2(0, 1)},
        TileId{tile.zoom_level + 1, tile.coords * 2u + glm::uvec2(1, 1)}};
}

TopDownTiler::TopDownTiler(const ctb::Grid& grid, const ctb::CRSBounds& bounds, Tile::Border border, Tile::Scheme scheme)
    : Tiler(grid, bounds, border, scheme)
{
}

std::vector<Tile> TopDownTiler::generateTiles(ctb::i_zoom zoom_level, ctb::TilePoint tilepoint) const
{
    const auto tile_ids = subtiles({ zoom_level, tilepoint });
    std::vector<Tile> tiles;
    for (const auto& tile_id : tile_ids) {
        Tile t;
        t.zoom = tile_id.zoom_level;
        t.point = tile_id.coords;
        t.tileSize = tile_size();
        t.gridSize = grid_size();
        tiles.push_back(std::move(t));
    }

    return tiles;
}
