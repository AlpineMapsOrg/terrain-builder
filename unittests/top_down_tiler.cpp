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

#include <catch2/catch.hpp>
#include <fmt/core.h>

#include "ParallelTiler.h"
#include "TopDownTiler.h"
#include "ctb/GlobalGeodetic.hpp"
#include "ctb/GlobalMercator.hpp"

TEMPLATE_TEST_CASE("BottomUpTiler, using tms scheme", "", std::true_type, std::false_type)
{
    SECTION("mercator / level 0")
    {
        const auto grid = ctb::GlobalMercator();
        const auto tiler = TopDownTiler(grid, grid.getExtent(), Tile::Tile::Border::No, TestType::value ? Tile::Tile::Scheme::Tms : Tile::Tile::Scheme::SlippyMap);

        const auto tiles = tiler.generateTiles(0, { 0, 0 });
        REQUIRE(tiles.size() == 4);
        {
            const auto parallel_tiler = ParallelTiler(grid, grid.getExtent(), Tile::Border::No, TestType::value ? Tile::Scheme::Tms : Tile::Scheme::SlippyMap);
            auto parallel_tiles = parallel_tiler.generateTiles(1);

            for (const auto& tile : tiles) {
                CHECK(tile.zoom == 1);
                REQUIRE(tile.point.x >= 0);
                REQUIRE(tile.point.x < 4);
                REQUIRE(tile.point.y >= 0);
                REQUIRE(tile.point.y < 4);

                const auto matching_tile_iter = std::find_if(parallel_tiles.begin(), parallel_tiles.end(), [&](const Tile& t) {
                    return t.point == tile.point;
                });
                REQUIRE(matching_tile_iter != parallel_tiles.end());

                CHECK(tile.tileSize == matching_tile_iter->tileSize);
                CHECK(tile.gridSize == matching_tile_iter->gridSize);

                parallel_tiles.erase(matching_tile_iter);
            }
        }

    }
}
