#include <catch2/catch.hpp>
#include <vector>

#include "catch2_helpers.h"
#include "Exception.h"
#include "MetaDataGenerator.h"
#include "Tiler.h"
#include "ctb/Grid.hpp"
#include "ctb/types.hpp"

template <typename TestType>
Tiler::Scheme tilerScheme() {
  return TestType::value ? Tiler::Scheme::Tms : Tiler::Scheme::SlippyMap;
}

template <typename TestType>
ctb::i_tile yCoord(ctb::i_tile tmsYCoord, ctb::i_tile nYTiles) {
  switch (tilerScheme<TestType>()) {
  case Tiler::Scheme::Tms:
    return tmsYCoord;
  case Tiler::Scheme::SlippyMap:
    return nYTiles - tmsYCoord - 1;
  }
  throw Exception("Not implemented!");
}

TEMPLATE_TEST_CASE("meta data generator, using tms scheme", "", std::true_type, std::false_type)  {
  const auto metadata = MetaDataGenerator::make(ATB_TEST_DATA_DIR "/austria/at_mgi.tif", ctb::Grid::Srs::SphericalMercator, tilerScheme<TestType>());

  SECTION("basics") {
    OGRSpatialReference webmercator;
    webmercator.importFromEPSG(3857);
    webmercator.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    CHECK(metadata.grid().getSRS().IsSame(&webmercator) == true);
    CHECK(metadata.tiler().scheme() == tilerScheme<TestType>());
  }

  SECTION("tile list") {
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
