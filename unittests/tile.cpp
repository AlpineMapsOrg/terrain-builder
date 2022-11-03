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

TEST_CASE("tile::Id scheme conversion")
{
    SECTION("tms -> slippy map")
    {
        CHECK(tile::Id{ 0, { 0, 0 }, tile::Scheme::Tms }.to(tile::Scheme::SlippyMap) == tile::Id{ 0, { 0, 0 }, tile::Scheme::SlippyMap });
        CHECK(tile::Id{ 1, { 0, 0 }, tile::Scheme::Tms }.to(tile::Scheme::SlippyMap) == tile::Id{ 1, { 0, 1 }, tile::Scheme::SlippyMap });
        CHECK(tile::Id{ 1, { 1, 1 }, tile::Scheme::Tms }.to(tile::Scheme::SlippyMap) == tile::Id{ 1, { 1, 0 }, tile::Scheme::SlippyMap });
        CHECK(tile::Id{ 2, { 2, 3 }, tile::Scheme::Tms }.to(tile::Scheme::SlippyMap) == tile::Id{ 2, { 2, 0 }, tile::Scheme::SlippyMap });
        CHECK(tile::Id{ 2, { 3, 1 }, tile::Scheme::Tms }.to(tile::Scheme::SlippyMap) == tile::Id{ 2, { 3, 2 }, tile::Scheme::SlippyMap });
    }
    SECTION("slippy map -> tms")
    {
        CHECK(tile::Id{ 0, { 0, 0 }, tile::Scheme::SlippyMap }.to(tile::Scheme::Tms) == tile::Id{ 0, { 0, 0 }, tile::Scheme::Tms });
        CHECK(tile::Id{ 1, { 0, 0 }, tile::Scheme::SlippyMap }.to(tile::Scheme::Tms) == tile::Id{ 1, { 0, 1 }, tile::Scheme::Tms });
        CHECK(tile::Id{ 1, { 1, 1 }, tile::Scheme::SlippyMap }.to(tile::Scheme::Tms) == tile::Id{ 1, { 1, 0 }, tile::Scheme::Tms });
        CHECK(tile::Id{ 2, { 2, 3 }, tile::Scheme::SlippyMap }.to(tile::Scheme::Tms) == tile::Id{ 2, { 2, 0 }, tile::Scheme::Tms });
        CHECK(tile::Id{ 2, { 3, 1 }, tile::Scheme::SlippyMap }.to(tile::Scheme::Tms) == tile::Id{ 2, { 3, 2 }, tile::Scheme::Tms });
    }
    SECTION("tms -> tms (no op)")
    {
        CHECK(tile::Id{ 0, { 0, 0 }, tile::Scheme::Tms }.to(tile::Scheme::Tms) == tile::Id{ 0, { 0, 0 }, tile::Scheme::Tms });
        CHECK(tile::Id{ 1, { 0, 0 }, tile::Scheme::Tms }.to(tile::Scheme::Tms) == tile::Id{ 1, { 0, 0 }, tile::Scheme::Tms });
        CHECK(tile::Id{ 1, { 1, 1 }, tile::Scheme::Tms }.to(tile::Scheme::Tms) == tile::Id{ 1, { 1, 1 }, tile::Scheme::Tms });
        CHECK(tile::Id{ 2, { 2, 3 }, tile::Scheme::Tms }.to(tile::Scheme::Tms) == tile::Id{ 2, { 2, 3 }, tile::Scheme::Tms });
        CHECK(tile::Id{ 2, { 3, 1 }, tile::Scheme::Tms }.to(tile::Scheme::Tms) == tile::Id{ 2, { 3, 1 }, tile::Scheme::Tms });
    }
    SECTION("slippy map -> slippy map (no op)")
    {
        CHECK(tile::Id{ 0, { 0, 0 }, tile::Scheme::SlippyMap }.to(tile::Scheme::SlippyMap) == tile::Id{ 0, { 0, 0 }, tile::Scheme::SlippyMap });
        CHECK(tile::Id{ 1, { 0, 0 }, tile::Scheme::SlippyMap }.to(tile::Scheme::SlippyMap) == tile::Id{ 1, { 0, 0 }, tile::Scheme::SlippyMap });
        CHECK(tile::Id{ 1, { 1, 1 }, tile::Scheme::SlippyMap }.to(tile::Scheme::SlippyMap) == tile::Id{ 1, { 1, 1 }, tile::Scheme::SlippyMap });
        CHECK(tile::Id{ 2, { 2, 3 }, tile::Scheme::SlippyMap }.to(tile::Scheme::SlippyMap) == tile::Id{ 2, { 2, 3 }, tile::Scheme::SlippyMap });
        CHECK(tile::Id{ 2, { 3, 1 }, tile::Scheme::SlippyMap }.to(tile::Scheme::SlippyMap) == tile::Id{ 2, { 3, 1 }, tile::Scheme::SlippyMap });
    }
}


TEST_CASE("tile::Id parent")
{
    CHECK(tile::Id { 1, { 0, 1 } }.parent() == tile::Id { 0, { 0, 0 } });
    CHECK(tile::Id { 2, { 2, 1 } }.parent() == tile::Id { 1, { 1, 0 } });
    CHECK(tile::Id { 2, { 0, 0 } }.parent() == tile::Id { 1, { 0, 0 } });
    CHECK(tile::Id { 2, { 3, 3 } }.parent() == tile::Id { 1, { 1, 1 } });
}

TEST_CASE("tile::Id, children") {
    SECTION("mercator tms (y point up)")
    {
        {
            const auto tiles = tile::Id{0, {0, 0}}.children();
            REQUIRE(tiles.size() == 4);
            for (const auto& tid : tiles)
                CHECK(tid.zoom_level == 1);
            CHECK(tiles[0].coords == glm::uvec2 { 0, 0 });
            CHECK(tiles[1].coords == glm::uvec2 { 1, 0 });
            CHECK(tiles[2].coords == glm::uvec2 { 0, 1 });
            CHECK(tiles[3].coords == glm::uvec2 { 1, 1 });
        }
        {
            const auto tiles = tile::Id{1, {1, 1}}.children();
            REQUIRE(tiles.size() == 4);
            for (const auto& tid : tiles)
                CHECK(tid.zoom_level == 2);
            CHECK(tiles[0].coords == glm::uvec2 { 2, 2 });
            CHECK(tiles[1].coords == glm::uvec2 { 3, 2 });
            CHECK(tiles[2].coords == glm::uvec2 { 2, 3 });
            CHECK(tiles[3].coords == glm::uvec2 { 3, 3 });
        }
    }
    SECTION("mercator SlippyMap (y points down)")
    {
        {
            const auto tiles = tile::Id{0, {0, 0}, tile::Scheme::SlippyMap}.children();
            REQUIRE(tiles.size() == 4);
            for (const auto& tid : tiles)
                CHECK(tid.zoom_level == 1);
            CHECK(tiles[0].coords == glm::uvec2 { 0, 1 });
            CHECK(tiles[1].coords == glm::uvec2 { 1, 1 });
            CHECK(tiles[2].coords == glm::uvec2 { 0, 0 });
            CHECK(tiles[3].coords == glm::uvec2 { 1, 0 });
        }
        {
            const auto tiles = tile::Id{1, {1, 0}, tile::Scheme::SlippyMap}.children();
            REQUIRE(tiles.size() == 4);
            for (const auto& tid : tiles)
                CHECK(tid.zoom_level == 2);
            CHECK(tiles[0].coords == glm::uvec2 { 2, 1 });
            CHECK(tiles[1].coords == glm::uvec2 { 3, 1 });
            CHECK(tiles[2].coords == glm::uvec2 { 2, 0 });
            CHECK(tiles[3].coords == glm::uvec2 { 3, 0 });
        }
    }

    SECTION("geodetic tms (y point up)")
    {

        {
            const auto tiles = tile::Id{0, {1, 0}, tile::Scheme::Tms}.children();
            REQUIRE(tiles.size() == 4);
            for (const auto& tid : tiles)
                CHECK(tid.zoom_level == 1);
            CHECK(tiles[0].coords == glm::uvec2 { 2, 0 });
            CHECK(tiles[1].coords == glm::uvec2 { 3, 0 });
            CHECK(tiles[2].coords == glm::uvec2 { 2, 1 });
            CHECK(tiles[3].coords == glm::uvec2 { 3, 1 });
        }
        {
            const auto tiles = tile::Id{1, {3, 0}, tile::Scheme::Tms}.children();
            REQUIRE(tiles.size() == 4);
            for (const auto& tid : tiles)
                CHECK(tid.zoom_level == 2);
            CHECK(tiles[0].coords == glm::uvec2 { 6, 0 });
            CHECK(tiles[1].coords == glm::uvec2 { 7, 0 });
            CHECK(tiles[2].coords == glm::uvec2 { 6, 1 });
            CHECK(tiles[3].coords == glm::uvec2 { 7, 1 });
        }
    }

    SECTION("geodetic SlippyMap (y points down)")
    {
        {
            const auto tiles = tile::Id{0, {1, 0}, tile::Scheme::SlippyMap}.children();
            REQUIRE(tiles.size() == 4);
            for (const auto& tid : tiles)
                CHECK(tid.zoom_level == 1);
            CHECK(tiles[0].coords == glm::uvec2 { 2, 1 });
            CHECK(tiles[1].coords == glm::uvec2 { 3, 1 });
            CHECK(tiles[2].coords == glm::uvec2 { 2, 0 });
            CHECK(tiles[3].coords == glm::uvec2 { 3, 0 });
        }
        {
            const auto tiles = tile::Id{1, {3, 1}, tile::Scheme::SlippyMap}.children();
            REQUIRE(tiles.size() == 4);
            for (const auto& tid : tiles)
                CHECK(tid.zoom_level == 2);
            CHECK(tiles[0].coords == glm::uvec2 { 6, 3 });
            CHECK(tiles[1].coords == glm::uvec2 { 7, 3 });
            CHECK(tiles[2].coords == glm::uvec2 { 6, 2 });
            CHECK(tiles[3].coords == glm::uvec2 { 7, 2 });
        }
    }

}
