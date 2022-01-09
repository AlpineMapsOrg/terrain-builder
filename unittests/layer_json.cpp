#include <catch2/catch.hpp>

#include "ctb/GlobalGeodetic.hpp"
#include "ctb/GlobalMercator.hpp"
#include "ctb/types.hpp"
#include "layer_json_writer.h"
#include "MetaDataGenerator.h"
#include "Tiler.h"
#include "ctb/Grid.hpp"

TEST_CASE("layer.json writer") {

//  const auto metadata = MetaDataGenerator::make(ATB_TEST_DATA_DIR "/austria/at_mgi.tif", ctb::Grid::Srs::SphericalMercator, Tiler::Scheme::Tms);

  SECTION("components") {
    CHECK(layer_json_writer::internal::tilejson("1.2.3") == R"("tilejson": "1.2.3",)");
    CHECK(layer_json_writer::internal::name("blah") == R"("name": "blah",)");
    CHECK(layer_json_writer::internal::description("bluh") == R"("description": "bluh",)");
    CHECK(layer_json_writer::internal::version("1.0.1") == R"("version": "1.0.1",)");
    CHECK(layer_json_writer::internal::format() == R"("format": "quantized-mesh-1.0",)");

    CHECK(layer_json_writer::internal::attribution("bleh") == R"("attribution": "bleh",)");
    CHECK(layer_json_writer::internal::schema(Tiler::Scheme::Tms) == R"("schema": "tms",)");
    CHECK(layer_json_writer::internal::schema(Tiler::Scheme::SlippyMap) == R"("schema": "slippyMap",)");
    CHECK(layer_json_writer::internal::tiles() == R"("tiles": [ "{z}/{x}/{y}.terrain?v={version}" ],)");
    CHECK(layer_json_writer::internal::projection(3857) == R"("projection": "EPSG:3857",)");
    CHECK(layer_json_writer::internal::bounds(ctb::GlobalMercator()) == R"("bounds": [-20037508.342789244, -20037508.342789244, 20037508.342789244, 20037508.342789244],)");
    CHECK(layer_json_writer::internal::bounds(ctb::GlobalGeodetic(256)) == R"("bounds": [-180, -90, 180, 90],)");

    CHECK(layer_json_writer::internal::zoom_layer(ctb::TileBounds(1, 2, 3, 4)) == R"([ { "startX": 1, "startY": 2, "endX": 3, "endY": 4 } ],)");
  }
}
