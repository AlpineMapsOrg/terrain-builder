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

#include "BottomUpTiler.h"
#include "ParallelTiler.h"

#include "ctb/GlobalMercator.hpp"
#include "ctb/GlobalGeodetic.hpp"


TEMPLATE_TEST_CASE("BottomUpTiler, using tms scheme", "", std::true_type, std::false_type) {
    SECTION("mercator / level 0") {
        const auto grid = ctb::GlobalMercator();
        const auto tiler = BottomUpTiler(grid, grid.getExtent(), Tile::Tile::Border::No, TestType::value ? Tile::Tile::Scheme::Tms : Tile::Tile::Scheme::SlippyMap);
    }
}
