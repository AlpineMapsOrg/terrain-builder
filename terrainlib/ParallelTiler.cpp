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

#include "ParallelTiler.h"

#include "Exception.h"
#include <functional>

ParallelTiler::ParallelTiler(const ctb::Grid& grid, const ctb::CRSBounds& bounds, Tile::Border border, Tile::Scheme scheme) : Tiler(grid, bounds, border, scheme)
{
}

ctb::TileCoordinate ParallelTiler::southWestTile(ctb::i_zoom zoom_level) const
{
    return convertToTilerScheme(grid().crsToTile(bounds().getLowerLeft(), zoom_level), n_y_tiles(zoom_level));
}

ctb::TileCoordinate ParallelTiler::northEastTile(ctb::i_zoom zoom_level) const
{
    const auto epsilon = grid().resolution(zoom_level) / 100;
    return convertToTilerScheme(grid().crsToTile(bounds().getUpperRight() - epsilon, zoom_level), n_y_tiles(zoom_level));
}

std::vector<Tile> ParallelTiler::generateTiles(ctb::i_zoom zoom_level) const
{

    const auto sw = grid().crsToTile(bounds().getLowerLeft(), zoom_level);
    const auto epsilon = grid().resolution(zoom_level) / 100;
    const auto ne = grid().crsToTile(bounds().getUpperRight() - epsilon, zoom_level);

    const ctb::i_tile n_y_tiles = this->n_y_tiles(zoom_level);
    //  const auto schemeTyFromInternalTy = (m_scheme == Scheme::Tms) ? ([](ctb::i_tile ty) -> ctb::i_tile { return ty; }) : ([n_y_tiles](ctb::i_tile ty) -> ctb::i_tile { return n_y_tiles - ty; });

    std::vector<Tile> tiles;
    tiles.reserve((ne.y - sw.y + 1) * (ne.x - sw.x + 1));
    for (auto ty = sw.y; ty <= ne.y; ++ty) {
        for (auto tx = sw.x; tx <= ne.x; ++tx) {
            const auto ty_p = (scheme() == Tile::Scheme::Tms) ? ty : n_y_tiles - ty - 1;
            const auto point = ctb::TilePoint { tx, ty_p };
            ctb::CRSBounds srs_bounds = grid().srsBounds(ctb::TileCoordinate(zoom_level, tx, ty), border_south_east() == Tile::Border::Yes);
            srs_bounds.clampBy(grid().getExtent());

            tiles.emplace_back(point, zoom_level, srs_bounds, grid().getEpsgCode(), grid_size(), tile_size());
            if (tiles.size() >= 1'000'000'000)
                // think about creating an on the fly tile generator. storing so many tiles takes a lot of memory.
                throw Exception("Setting the zoom level so higher is probably not a good idea. This would generate more than 1'000 million tiles. "
                                "I'm aborting. If you really need this, then that means that the future is bright. But you'll have to edit the code..\n"
                                "      .   .     \n     / \\_/ \\    \n    | O  O |   \n    | ~V~  |  _\n     ~_ _ /  // \n    /     \\ //  \n"
                                "    |  ||  |/  \n    | /  \\ |   \n    ||    ||   \n               \n");
        }
    }

    return tiles;
}

std::vector<Tile> ParallelTiler::generateTiles(const std::pair<ctb::i_zoom, ctb::i_zoom>& zoom_range) const
{
    std::vector<Tile> tiles;
    assert(zoom_range.first <= zoom_range.second);
    for (ctb::i_zoom i = zoom_range.first; i <= zoom_range.second; ++i) {
        auto zoom_level_tiles = generateTiles(i);
        tiles.reserve(tiles.size() + zoom_level_tiles.size());
        std::move(zoom_level_tiles.begin(), zoom_level_tiles.end(), std::back_inserter(tiles));
    }
    return tiles;
}
