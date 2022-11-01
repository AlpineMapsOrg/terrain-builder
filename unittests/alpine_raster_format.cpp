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

#include <filesystem>

#include <catch2/catch.hpp>
#include <fmt/core.h>

#include "Dataset.h"
#include "Image.h"
#include "ParallelTiler.h"
#include "alpine_raster.h"
#include "catch2_helpers.h"
#include "ctb/Grid.hpp"
#include "ctb/types.hpp"

TEST_CASE("alpine raster format conversion math")
{
    const auto one_red = 32.0f;
    const auto one_green = 32.000000001f / 256;
    const auto eps = 0.000000001f;

    CHECK(alpine_raster::convert(0) == glm::u8vec3(0, 0, 0));

    CHECK(alpine_raster::convert(one_red + eps) == glm::u8vec3(1, 0, 0));
    CHECK(alpine_raster::convert(one_green + eps) == glm::u8vec3(0, 1, 0));

    CHECK(alpine_raster::convert(42 * one_red + eps) == glm::u8vec3(42, 0, 0));
    CHECK(alpine_raster::convert(200 * one_green + eps) == glm::u8vec3(0, 200, 0));

    CHECK(alpine_raster::convert(255 * one_red + eps) == glm::u8vec3(255, 0, 0));
    CHECK(alpine_raster::convert(253 * one_green + eps) == glm::u8vec3(0, 253, 0));
    CHECK(alpine_raster::convert(254 * one_green + eps) == glm::u8vec3(0, 254, 0));
    CHECK(alpine_raster::convert(255 * one_green + eps) == glm::u8vec3(0, 255, 0));
    CHECK(alpine_raster::convert(256 * one_green + eps) == glm::u8vec3(1, 0, 0));
    CHECK(alpine_raster::convert(257 * one_green + eps) == glm::u8vec3(1, 1, 0));
    CHECK(alpine_raster::convert(258 * one_green + eps) == glm::u8vec3(1, 2, 0));

    CHECK(alpine_raster::convert(240 * one_red + 195 * one_green + eps) == glm::u8vec3(240, 195, 0));
    CHECK(alpine_raster::convert(64 * one_red + 255 * one_green + eps) == glm::u8vec3(64, 255, 0));
    CHECK(alpine_raster::convert(255 * one_red + 128 * one_green + eps) == glm::u8vec3(255, 128, 0));
    CHECK(alpine_raster::convert(255 * one_red + 255 * one_green + eps) == glm::u8vec3(255, 255, 0));

    CHECK(alpine_raster::convert(123 * one_red + 250 * one_green + eps) == glm::u8vec3(123, 250, 0));
    CHECK(alpine_raster::convert(140 * one_red + 255 * one_green + eps) == glm::u8vec3(140, 255, 0));
    CHECK(alpine_raster::convert(141 * one_red + 0 * one_green + eps) == glm::u8vec3(141, 0, 0));
    CHECK(alpine_raster::convert(141 * one_red + 1 * one_green + eps) == glm::u8vec3(141, 1, 0));
}

Tile::Border testTypeValue2Border(bool v)
{
    return v ? Tile::Border::Yes : Tile::Border::No;
}

TEMPLATE_TEST_CASE("alpine raster format, border ", "", std::true_type, std::false_type)
{
    std::filesystem::remove_all("./unittest_tiles/");

    SECTION("raste write")
    {
        const auto generator = alpine_raster::make_generator(ATB_TEST_DATA_DIR "/austria/at_mgi.tif", "./unittest_tiles/", ctb::Grid::Srs::SphericalMercator, Tile::Scheme::Tms, Tile::Border::Yes);
        generator.write(Tile { {0, ctb::TilePoint(0, 0)}, {}, int(ctb::Grid::Srs::SphericalMercator), 256, 257 }, HeightData(257, 257));
        CHECK(std::filesystem::exists("./unittest_tiles/0/0/0.png"));

        generator.write(Tile { {1, ctb::TilePoint(2, 3)}, {}, int(ctb::Grid::Srs::SphericalMercator), 256, 257 }, HeightData(257, 257));
        CHECK(std::filesystem::exists("./unittest_tiles/1/2/3.png"));

        // check that a second write doesn't crash
        generator.write(Tile { {1, ctb::TilePoint(2, 3)}, {}, int(ctb::Grid::Srs::SphericalMercator), 256, 257 }, HeightData(257, 257));
        CHECK(std::filesystem::exists("./unittest_tiles/1/2/3.png"));

        // in the best case, we would read back the data and check it. but that's too much work for now.
        // it'll be checked in cesium.js
    }

    SECTION("process all tiles")
    {
        auto generator = alpine_raster::make_generator(ATB_TEST_DATA_DIR "/austria/at_mgi.tif", "./unittest_tiles/", ctb::Grid::Srs::SphericalMercator, Tile::Scheme::Tms, testTypeValue2Border(TestType::value));
        generator.setWarnOnMissingOverviews(false);
        generator.process({ 0, 7 });
        const auto tiles = generator.tiler().generateTiles({ 0, 7 });

        const auto check = [](const Tile& t) {
            return std::filesystem::exists(fmt::format("./unittest_tiles/{}/{}/{}.png", t.tile_id.zoom_level, t.tile_id.coords.x, t.tile_id.coords.y));
        };
        CHECK(std::transform_reduce(tiles.begin(), tiles.end(), true, std::logical_and<>(), check) == true);
    }
#if defined(ATB_UNITTESTS_EXTENDED) && ATB_UNITTESTS_EXTENDED
    SECTION("process all tiles with max zoom")
    {
        auto generator = alpine_raster::make_generator(ATB_TEST_DATA_DIR "/austria/at_mgi.tif", "./unittest_tiles/", ctb::Grid::Srs::SphericalMercator, Tile::Scheme::Tms, testTypeValue2Border(TestType::value));
        generator.setWarnOnMissingOverviews(false);
        generator.process({ 4, 8 });
        const auto tiles = generator.tiler().generateTiles({ 4, 8 });

        const auto check = [](const Tile& t) {
            return std::filesystem::exists(fmt::format("./unittest_tiles/{}/{}/{}.png", t.tile_id.zoom_level, t.tile_id.coords.x, t.tile_id.coords.y));
        };
        CHECK(std::transform_reduce(tiles.begin(), tiles.end(), true, std::logical_and<>(), check) == true);
    }
#endif
}
