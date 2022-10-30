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

#include <catch2/catch.hpp>
#include <vector>

#include "Exception.h"
#include "MetaDataGenerator.h"
#include "ParallelTiler.h"
#include "catch2_helpers.h"
#include "ctb/Grid.hpp"
#include "ctb/types.hpp"

template <typename TestType>
Tile::Scheme tilerScheme()
{
    return TestType::value ? Tile::Scheme::Tms : Tile::Scheme::SlippyMap;
}

template <typename TestType>
ctb::i_tile yCoord(ctb::i_tile tmsYCoord, ctb::i_tile nYTiles)
{
    switch (tilerScheme<TestType>()) {
    case Tile::Scheme::Tms:
        return tmsYCoord;
    case Tile::Scheme::SlippyMap:
        return nYTiles - tmsYCoord - 1;
    }
    throw Exception("Not implemented!");
}

TEMPLATE_TEST_CASE("meta data generator, using tms scheme", "", std::true_type, std::false_type)
{
    const auto metadata = MetaDataGenerator::make(ATB_TEST_DATA_DIR "/austria/at_mgi.tif", ctb::Grid::Srs::SphericalMercator, tilerScheme<TestType>());

    SECTION("basics")
    {
        OGRSpatialReference webmercator;
        webmercator.importFromEPSG(3857);
        webmercator.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

        CHECK(metadata.grid().getSRS().IsSame(&webmercator) == true);
        CHECK(metadata.tiler().scheme() == tilerScheme<TestType>());
    }

    SECTION("tile list")
    {
        const std::vector<ctb::TileBounds> available_tile_list = metadata.availableTiles();
        REQUIRE(available_tile_list.size() == 7);
        CHECK(available_tile_list[0].getLowerLeft() == ctb::TilePoint(0, 0));
        CHECK(available_tile_list[0].getUpperRight() == ctb::TilePoint(0, 0));

        CHECK(available_tile_list[1].getLowerLeft() == ctb::TilePoint(1, yCoord<TestType>(1, 2)));
        CHECK(available_tile_list[1].getUpperRight() == ctb::TilePoint(1, yCoord<TestType>(1, 2)));

        CHECK(available_tile_list[2].getLowerLeft() == ctb::TilePoint(2, yCoord<TestType>(2, 4)));
        CHECK(available_tile_list[2].getUpperRight() == ctb::TilePoint(2, yCoord<TestType>(2, 4)));

        CHECK(available_tile_list[3].getLowerLeft() == ctb::TilePoint(4, yCoord<TestType>(5, 8)));
        CHECK(available_tile_list[3].getUpperRight() == ctb::TilePoint(4, yCoord<TestType>(5, 8)));

        CHECK(available_tile_list[4].getLowerLeft() == ctb::TilePoint(8, yCoord<TestType>(10, 16)));
        CHECK(available_tile_list[4].getUpperRight() == ctb::TilePoint(8, yCoord<TestType>(10, 16)));

        CHECK(available_tile_list[5].getLowerLeft() == ctb::TilePoint(16, yCoord<TestType>(20, 32)));
        CHECK(available_tile_list[5].getUpperRight() == ctb::TilePoint(17, yCoord<TestType>(21, 32)));

        CHECK(available_tile_list[6].getLowerLeft() == ctb::TilePoint(33, yCoord<TestType>(41, 64)));
        CHECK(available_tile_list[6].getUpperRight() == ctb::TilePoint(35, yCoord<TestType>(42, 64)));
    }
}
