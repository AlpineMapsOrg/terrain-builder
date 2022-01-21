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

#include "Dataset.h"
#include "DatasetReader.h"
#include "Tile.h"
#include "Image.h"
#include "cesium_tin_terra.h"
#include "util.h"
#include "tntn/Mesh.h"


TEST_CASE("tin terra write") {

  SECTION("cesium terrain") {
    const auto generator = cesium_tin_terra::make_generator("./unittest_tiles/", ATB_TEST_DATA_DIR "/austria/at_mgi.tif", ctb::Grid::Srs::SphericalMercator, Tiler::Scheme::Tms, Tiler::Border::No);
    generator.write(Tile{ctb::TilePoint(0, 0), 0, {0, 0, 255, 255}, 256, 256}, HeightData(256, 256));
    CHECK(std::filesystem::exists("./unittest_tiles/0/0/0.terrain"));
  }

  SECTION("mesh vertex ranges") {
    const auto converter = cesium_tin_terra::TileWriter(Tiler::Border::No);
    const auto at_bounds = ctb::CRSBounds(9.5, 46.4, 17.1, 49.0);
    OGRSpatialReference webmercator;
    webmercator.importFromEPSG(3857);
    webmercator.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    OGRSpatialReference wgs84;
    wgs84.importFromEPSG(4326);
    wgs84.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    const auto at_webmercator_bounds = util::nonExactBoundsTransform(at_bounds, wgs84, webmercator);

    const auto dataset = Dataset::make_shared(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");
    const auto reader = DatasetReader(dataset, webmercator, 1);
    const auto heights = reader.read(at_webmercator_bounds, 256, 256);

    const auto mesh = converter.toMesh(at_webmercator_bounds, heights);
    REQUIRE(mesh->vertices().distance() > 0);
  }

  SECTION("obj debug terrain") {
    const auto generator = cesium_tin_terra::make_objGenerator("./debugtest_tiles/", ATB_TEST_DATA_DIR "/austria/at_mgi.tif", ctb::Grid::Srs::SphericalMercator, Tiler::Scheme::Tms, Tiler::Border::Yes);
    generator.process({5, 7});
  }

}
