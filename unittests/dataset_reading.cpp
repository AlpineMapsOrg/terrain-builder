#include <catch2/catch.hpp>
#include <fmt/core.h>
#include <ogr_spatialref.h>

#include "Dataset.h"
#include "DatasetReader.h"
#include "HeightData.h"
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

  SECTION("read without overviews") {
    const auto dataset = std::make_shared<Dataset>(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");
    OGRSpatialReference geodetic;
    geodetic.importFromEPSG(4326);
    geodetic.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    const auto reader = DatasetReader(dataset, geodetic);
    const auto vienna_bounds = ctb::CRSBounds(16.17, 48.11, 16.59, 48.33);
    const auto heights = reader.read(vienna_bounds, 42, 420);
    REQUIRE(heights.width() == 42);
    REQUIRE(heights.height() == 420);
    REQUIRE(heights.pixel(21, 200) > 0);
  }
}


