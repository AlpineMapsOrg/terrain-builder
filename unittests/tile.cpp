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

#include "Tile.h"

TEST_CASE("Tile::Id scheme conversion")
{
    SECTION("tms -> slippy map")
    {
        CHECK(Tile::Id{ 0, { 0, 0 }, Tile::Scheme::Tms }.to(Tile::Scheme::SlippyMap) == Tile::Id{ 0, { 0, 0 }, Tile::Scheme::SlippyMap });
        CHECK(Tile::Id{ 1, { 0, 0 }, Tile::Scheme::Tms }.to(Tile::Scheme::SlippyMap) == Tile::Id{ 1, { 0, 1 }, Tile::Scheme::SlippyMap });
        CHECK(Tile::Id{ 1, { 1, 1 }, Tile::Scheme::Tms }.to(Tile::Scheme::SlippyMap) == Tile::Id{ 1, { 1, 0 }, Tile::Scheme::SlippyMap });
        CHECK(Tile::Id{ 2, { 2, 3 }, Tile::Scheme::Tms }.to(Tile::Scheme::SlippyMap) == Tile::Id{ 2, { 2, 0 }, Tile::Scheme::SlippyMap });
        CHECK(Tile::Id{ 2, { 3, 1 }, Tile::Scheme::Tms }.to(Tile::Scheme::SlippyMap) == Tile::Id{ 2, { 3, 2 }, Tile::Scheme::SlippyMap });
    }
    SECTION("slippy map -> tms")
    {
        CHECK(Tile::Id{ 0, { 0, 0 }, Tile::Scheme::SlippyMap }.to(Tile::Scheme::Tms) == Tile::Id{ 0, { 0, 0 }, Tile::Scheme::Tms });
        CHECK(Tile::Id{ 1, { 0, 0 }, Tile::Scheme::SlippyMap }.to(Tile::Scheme::Tms) == Tile::Id{ 1, { 0, 1 }, Tile::Scheme::Tms });
        CHECK(Tile::Id{ 1, { 1, 1 }, Tile::Scheme::SlippyMap }.to(Tile::Scheme::Tms) == Tile::Id{ 1, { 1, 0 }, Tile::Scheme::Tms });
        CHECK(Tile::Id{ 2, { 2, 3 }, Tile::Scheme::SlippyMap }.to(Tile::Scheme::Tms) == Tile::Id{ 2, { 2, 0 }, Tile::Scheme::Tms });
        CHECK(Tile::Id{ 2, { 3, 1 }, Tile::Scheme::SlippyMap }.to(Tile::Scheme::Tms) == Tile::Id{ 2, { 3, 2 }, Tile::Scheme::Tms });
    }
    SECTION("tms -> tms (no op)")
    {
        CHECK(Tile::Id{ 0, { 0, 0 }, Tile::Scheme::Tms }.to(Tile::Scheme::Tms) == Tile::Id{ 0, { 0, 0 }, Tile::Scheme::Tms });
        CHECK(Tile::Id{ 1, { 0, 0 }, Tile::Scheme::Tms }.to(Tile::Scheme::Tms) == Tile::Id{ 1, { 0, 0 }, Tile::Scheme::Tms });
        CHECK(Tile::Id{ 1, { 1, 1 }, Tile::Scheme::Tms }.to(Tile::Scheme::Tms) == Tile::Id{ 1, { 1, 1 }, Tile::Scheme::Tms });
        CHECK(Tile::Id{ 2, { 2, 3 }, Tile::Scheme::Tms }.to(Tile::Scheme::Tms) == Tile::Id{ 2, { 2, 3 }, Tile::Scheme::Tms });
        CHECK(Tile::Id{ 2, { 3, 1 }, Tile::Scheme::Tms }.to(Tile::Scheme::Tms) == Tile::Id{ 2, { 3, 1 }, Tile::Scheme::Tms });
    }
    SECTION("slippy map -> slippy map (no op)")
    {
        CHECK(Tile::Id{ 0, { 0, 0 }, Tile::Scheme::SlippyMap }.to(Tile::Scheme::SlippyMap) == Tile::Id{ 0, { 0, 0 }, Tile::Scheme::SlippyMap });
        CHECK(Tile::Id{ 1, { 0, 0 }, Tile::Scheme::SlippyMap }.to(Tile::Scheme::SlippyMap) == Tile::Id{ 1, { 0, 0 }, Tile::Scheme::SlippyMap });
        CHECK(Tile::Id{ 1, { 1, 1 }, Tile::Scheme::SlippyMap }.to(Tile::Scheme::SlippyMap) == Tile::Id{ 1, { 1, 1 }, Tile::Scheme::SlippyMap });
        CHECK(Tile::Id{ 2, { 2, 3 }, Tile::Scheme::SlippyMap }.to(Tile::Scheme::SlippyMap) == Tile::Id{ 2, { 2, 3 }, Tile::Scheme::SlippyMap });
        CHECK(Tile::Id{ 2, { 3, 1 }, Tile::Scheme::SlippyMap }.to(Tile::Scheme::SlippyMap) == Tile::Id{ 2, { 3, 1 }, Tile::Scheme::SlippyMap });
    }
}


TEST_CASE("Tile::Id parent")
{
    CHECK(Tile::Id { 1, { 0, 1 } }.parent() == Tile::Id { 0, { 0, 0 } });
    CHECK(Tile::Id { 2, { 2, 1 } }.parent() == Tile::Id { 1, { 1, 0 } });
    CHECK(Tile::Id { 2, { 0, 0 } }.parent() == Tile::Id { 1, { 0, 0 } });
    CHECK(Tile::Id { 2, { 3, 3 } }.parent() == Tile::Id { 1, { 1, 1 } });

}
