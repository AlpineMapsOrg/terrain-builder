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

#include <regex>
#include <catch2/catch.hpp>

#include "ctb/GlobalGeodetic.hpp"
#include "ctb/GlobalMercator.hpp"
#include "ctb/types.hpp"
#include "layer_json_writer.h"
#include "MetaDataGenerator.h"
#include "ParallelTiler.h"
#include "ctb/Grid.hpp"

namespace {
std::string clean(std::string s) {
  return std::regex_replace(s, std::regex(R"([\n\r\s]+)"), "");
}
}

TEST_CASE("layer.json writer") {
  SECTION("components") {
    // actually, probably we shouldn't test these pieces (no public interface, can easily change in the future).
    // but it's already there, so leaving for now. delete it if you happen to change the json writer.
    CHECK(layer_json_writer::internal::tilejson("1.2.3") == R"("tilejson": "1.2.3",)");
    CHECK(layer_json_writer::internal::name("blah") == R"("name": "blah",)");
    CHECK(layer_json_writer::internal::description("bluh") == R"("description": "bluh",)");
    CHECK(layer_json_writer::internal::version("1.0.1") == R"("version": "1.0.1",)");
    CHECK(layer_json_writer::internal::format() == R"("format": "quantized-mesh-1.0",)");

    CHECK(layer_json_writer::internal::attribution("bleh") == R"("attribution": "bleh",)");
    CHECK(layer_json_writer::internal::schema(Tile::Scheme::Tms) == R"("schema": "tms",)");
    CHECK(layer_json_writer::internal::schema(Tile::Scheme::SlippyMap) == R"("schema": "slippyMap",)");
    CHECK(layer_json_writer::internal::tiles() == R"("tiles": [ "{z}/{x}/{y}.terrain?v={version}" ],)");
    CHECK(layer_json_writer::internal::projection(3857) == R"("projection": "EPSG:3857",)");
    CHECK(layer_json_writer::internal::bounds(ctb::GlobalMercator()) == R"("bounds": [-20037508.34, -20037508.34, 20037508.34, 20037508.34],)");
    CHECK(layer_json_writer::internal::bounds(ctb::GlobalGeodetic(256)) == R"("bounds": [-180.00, -90.00, 180.00, 90.00],)");

    CHECK(layer_json_writer::internal::zoom_layer(ctb::TileBounds(1, 2, 3, 4)) == R"([ { "startX": 1, "startY": 2, "endX": 3, "endY": 4 } ],)");
  }

  SECTION("austria webmercator tms") {
    const auto metadata = MetaDataGenerator::make(ATB_TEST_DATA_DIR "/austria/at_mgi.tif", ctb::Grid::Srs::SphericalMercator, Tile::Scheme::Tms);
    CHECK(clean(layer_json_writer::process(metadata)) == clean(R"({
            "tilejson": "2.1.0",
            "name": "at_mgi",
            "description": "",
            "version": "1.1.0",
            "format": "quantized-mesh-1.0",
            "attribution": "",
            "schema": "tms",
            "tiles": [ "{z}/{x}/{y}.terrain?v={version}" ],
            "projection": "EPSG:3857",
            "bounds": [ -20037508.34, -20037508.34, 20037508.34, 20037508.34 ],
            "available": [
             [ { "startX": 0, "startY": 0, "endX": 0, "endY": 0 } ],
             [ { "startX": 1, "startY": 1, "endX": 1, "endY": 1 } ],
             [ { "startX": 2, "startY": 2, "endX": 2, "endY": 2 } ],
             [ { "startX": 4, "startY": 5, "endX": 4, "endY": 5 } ],
             [ { "startX": 8, "startY": 10, "endX": 8, "endY": 10 } ],
             [ { "startX": 16, "startY": 20, "endX": 17, "endY": 21 } ],
             [ { "startX": 33, "startY": 41, "endX": 35, "endY": 42 } ],
            ]
          })"));
  }
}
