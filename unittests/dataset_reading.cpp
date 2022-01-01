#include <catch2/catch.hpp>
#include <fmt/core.h>
#include <ogr_spatialref.h>

#include "Dataset.h"
#include "Tiler.h"
#include "ctb/GlobalMercator.hpp"
#include "ctb/GlobalGeodetic.hpp"

void checkBounds(const ctb::CRSBounds& a, const ctb::CRSBounds& b) {
  REQUIRE(a.getHeight() > 0);
  REQUIRE(a.getWidth() > 0);
  REQUIRE(b.getHeight() > 0);
  REQUIRE(b.getWidth() > 0);

  const auto heightErrorIn = std::abs(a.getHeight() - b.getHeight()) / a.getHeight();
  const auto widthErrorIn = std::abs(a.getWidth() - b.getWidth()) / a.getWidth();
//  fmt::print("height error = {}, width error = {}\n", heightErrorIn, widthErrorIn);
  REQUIRE(heightErrorIn < 0.001);
  REQUIRE(widthErrorIn < 0.001);
}

TEST_CASE("datasets are as expected") {
  auto d_mgi = Dataset(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");
  auto d_wgs84 = Dataset(ATB_TEST_DATA_DIR "/austria/at_wgs84.tif");

  OGRSpatialReference webmercator;
  webmercator.importFromEPSG(3857);
  webmercator.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  SECTION("SRS") {
    REQUIRE_FALSE(d_mgi.srs().IsEmpty());
    REQUIRE_FALSE(d_wgs84.srs().IsEmpty());
    const auto wgs84 = d_wgs84.srs();
    REQUIRE_FALSE(d_mgi.srs().IsSame(&wgs84));
  }

  SECTION("bounds") {
    {
//      fmt::print("webmercator: \n");
      checkBounds(d_mgi.bounds(webmercator), d_wgs84.bounds(webmercator));
    }
    {
//      fmt::print("d_wgs84: \n");
      checkBounds(d_mgi.bounds(d_wgs84.srs()), d_wgs84.bounds(d_wgs84.srs()));
    }
//    {
//      // doesn't work: wgs84 was created by projecting the original mgi data. since wgs84 is warped, the projection creates a border.
//      //               when backprojecting to mgi, there is still a border in the raster data (but widthout height information).
//      //               this is not an error in the code, and impossible to avoid when rerasterising warped raster data
//      fmt::print("d_mgi: \n");
//      checkBounds(d_mgi.bounds(d_mgi.srs()), d_wgs84.bounds(d_mgi.srs()));
//    }
  }
  SECTION("resolution") {
    REQUIRE(d_mgi.widthInPixels() == 620);
    REQUIRE(d_mgi.heightInPixels() == 350);
    REQUIRE(std::abs(d_mgi.pixelWidthIn(webmercator) - 573000.0/620) / 573000.0/620 < 0.01);
    REQUIRE(std::abs(d_mgi.pixelHeightIn(webmercator) - 294000.0/350) / 294000.0/350 < 0.01);
    const auto grid = ctb::GlobalMercator();
    const auto gridResuloution = std::min(d_mgi.pixelWidthIn(grid.getSRS()), d_mgi.pixelHeightIn(grid.getSRS()));
    REQUIRE(grid.zoomForResolution(gridResuloution) == 7);
  }
}



TEST_CASE("tiler bounds") {
//  const auto bounds = ctb::CRSBounds(1'000'000, 6'000'000, 2'000'000, 6'700'000); // in m
  SECTION("mercator / quantised mesh level 0") {
    const auto grid = ctb::GlobalMercator();
    const auto tiler = Tiler(grid, grid.getExtent(), Tiler::Border::No);

    const auto l0_tiles = tiler.generateTiles(0);
    REQUIRE(l0_tiles.size() == 1);
    const auto t = l0_tiles.front();
    REQUIRE(t.zoom == 0);
    REQUIRE(t.point == ctb::TilePoint(0, 0));
    REQUIRE(t.gridSize == 256);
    REQUIRE(t.tileSize == 256);

    // all extents are equal, because we have only one tile and no border
    REQUIRE(t.srsBounds.getMinX() == Approx(grid.getExtent().getMinX()));
    REQUIRE(t.srsBounds.getMaxY() == Approx(grid.getExtent().getMaxY()));
    REQUIRE(t.srsBounds.getMinY() == Approx(grid.getExtent().getMinY()));
    REQUIRE(t.srsBounds.getMaxX() == Approx(grid.getExtent().getMaxX()));
  }

  SECTION("mercator / quantised mesh level 1 and 2") {
    const auto grid = ctb::GlobalMercator();
    auto dataset = Dataset(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");
    const auto tiler = Tiler(grid, dataset.bounds(grid.getSRS()), Tiler::Border::No);

    {
      // https://www.maptiler.com/google-maps-coordinates-tile-bounds-projection/#1/-5.80/62.29
      // this code is for TMS mapping, i.e., tile y = 0 is south
      const auto l1_tiles = tiler.generateTiles(1);
      REQUIRE(l1_tiles.size() == 1);
      const auto t = l1_tiles.front();
      REQUIRE(t.zoom == 1);
      REQUIRE(t.point == ctb::TilePoint(1, 1));
      REQUIRE(t.gridSize == 256);
      REQUIRE(t.tileSize == 256);

      REQUIRE(t.srsBounds.getMinX() == Approx(0));
      REQUIRE(t.srsBounds.getMinY() == Approx(0));
      REQUIRE(t.srsBounds.getMaxY() == Approx(grid.getExtent().getMaxY()));
      REQUIRE(t.srsBounds.getMaxX() == Approx(grid.getExtent().getMaxX()));
    }
    {
      const auto l2_tiles = tiler.generateTiles(2);
      REQUIRE(l2_tiles.size() == 1);
      const auto t = l2_tiles.front();
      REQUIRE(t.zoom == 2);
      REQUIRE(t.point == ctb::TilePoint(2, 2));
      REQUIRE(t.gridSize == 256);
      REQUIRE(t.tileSize == 256);

      REQUIRE(t.srsBounds.getMinX() == Approx(0));
      REQUIRE(t.srsBounds.getMinY() == Approx(0));
      REQUIRE(t.srsBounds.getMaxY() == Approx(grid.getExtent().getMaxY() / 2));
      REQUIRE(t.srsBounds.getMaxX() == Approx(grid.getExtent().getMaxX() / 2));
    }
  }

  SECTION("geodetic / cesium terrain raster level 0") {
    const auto grid = ctb::GlobalGeodetic(64);
    const auto tiler = Tiler(grid, grid.getExtent(), Tiler::Border::Yes);

    const auto l0_tiles = tiler.generateTiles(0);
    REQUIRE(l0_tiles.size() == 2);
    const auto t0 = l0_tiles[0];
    REQUIRE(t0.zoom == 0);
    REQUIRE(t0.point == ctb::TilePoint(0, 0));
    REQUIRE(t0.gridSize == 64);
    REQUIRE(t0.tileSize == 65);
    // north, south and west are equal to the extents (no overflow)
    REQUIRE(t0.srsBounds.getMinY() == Approx(grid.getExtent().getMinY()));
    REQUIRE(t0.srsBounds.getMaxY() == Approx(grid.getExtent().getMaxY()));
    REQUIRE(t0.srsBounds.getMinX() == Approx(grid.getExtent().getMinX()));
    // east should be somewher close to the 0 meridian, 1 pixel east of it to be exact)
    REQUIRE(t0.srsBounds.getMaxX() == Approx(180.0 / 64));

    const auto t1 = l0_tiles[1];
    REQUIRE(t1.zoom == 0);
    REQUIRE(t1.point == ctb::TilePoint(1, 0));
    REQUIRE(t1.gridSize == 64);
    REQUIRE(t1.tileSize == 65);
    // north and south are equal to the extent (no overflow)
    REQUIRE(t1.srsBounds.getMinY() == Approx(grid.getExtent().getMinY()));
    REQUIRE(t1.srsBounds.getMaxY() == Approx(grid.getExtent().getMaxY()));
    // west is the 0 meriaidan
    REQUIRE(t1.srsBounds.getMinX() == Approx(0));
    // east should be wrapped around. but lets make things easier for now, and clamp it.
    // it goes mostly through the pacific anyways, and I wonder whether gdal even supports wrapping around the back..
    REQUIRE(t1.srsBounds.getMaxX() == Approx(grid.getExtent().getMaxX()));
  }

  SECTION("geodetic / cesium terrain raster level 1 and 2") {
    const auto grid = ctb::GlobalGeodetic(64);
    auto dataset = Dataset(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");
    const auto tiler = Tiler(grid, dataset.bounds(grid.getSRS()), Tiler::Border::Yes);

    {
      const auto l1_tiles = tiler.generateTiles(1);
      REQUIRE(l1_tiles.size() == 1);
      const auto t = l1_tiles.front();
      REQUIRE(t.zoom == 1);
      REQUIRE(t.point == ctb::TilePoint(2, 1));
      REQUIRE(t.gridSize == 64);
      REQUIRE(t.tileSize == 65);

      constexpr auto tileSizeX = 360 / 4.0;
      REQUIRE(t.srsBounds.getMinX() == Approx(0));
      REQUIRE(t.srsBounds.getMinY() == Approx(0));
      REQUIRE(t.srsBounds.getMaxX() == Approx(tileSizeX + tileSizeX / 64.0));
      REQUIRE(t.srsBounds.getMaxY() == Approx(90));
    }
    {
      const auto l2_tiles = tiler.generateTiles(2);
      REQUIRE(l2_tiles.size() == 1);
      const auto t = l2_tiles.front();
      REQUIRE(t.zoom == 2);
      REQUIRE(t.point == ctb::TilePoint(4, 3));
      REQUIRE(t.gridSize == 64);
      REQUIRE(t.tileSize == 65);

      constexpr auto tileSizeX = 360 / 8.0;
      constexpr auto tileSizeY = 180 / 4.0;
      REQUIRE(t.srsBounds.getMinX() == Approx(0));
      REQUIRE(t.srsBounds.getMinY() == Approx(tileSizeY));
      REQUIRE(t.srsBounds.getMaxX() == Approx(tileSizeX + tileSizeX / 64.0));
      REQUIRE(t.srsBounds.getMaxY() == Approx(90));
    }
  }
}
