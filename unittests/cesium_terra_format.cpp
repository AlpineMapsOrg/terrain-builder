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
#include "srs.h"
#include "tntn/Mesh.h"


TEST_CASE("tin terra write") {
  std::filesystem::remove_all("./unittest_tiles/");

  SECTION("cesium terrain") {
    const auto generator = cesium_tin_terra::make_generator(ATB_TEST_DATA_DIR "/austria/at_mgi.tif", "./unittest_tiles/", ctb::Grid::Srs::SphericalMercator, ParallelTiler::Scheme::Tms, ParallelTiler::Border::No);
    const auto b = 20037508.3427892;
    generator.write(Tile{ctb::TilePoint(0, 0), 0, {-b, -b, b, b}, int(ctb::Grid::Srs::SphericalMercator), 256, 257}, HeightData(257, 257));
    CHECK(std::filesystem::exists("./unittest_tiles/0/0/0.terrain"));
  }

  SECTION("mesh vertex ranges webmercator") {
    const auto converter = cesium_tin_terra::TileWriter(ParallelTiler::Border::No);
    const auto at_bounds = ctb::CRSBounds(9.5, 46.4, 17.1, 49.0);
    OGRSpatialReference webmercator;
    webmercator.importFromEPSG(3857);
    webmercator.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    OGRSpatialReference wgs84;
    wgs84.importFromEPSG(4326);
    wgs84.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    const auto at_webmercator_bounds = srs::nonExactBoundsTransform(at_bounds, wgs84, webmercator);
//    const auto at_webmercator_bounds = ctb::CRSBounds(1100000.0, 5900000.0, 1800000.0, 6200000.0);

    const auto dataset = Dataset::make_shared(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");
    const auto reader = DatasetReader(dataset, webmercator, 1, false);
    const auto heights = reader.read(at_webmercator_bounds, 257, 257);

    const auto mesh = converter.toMesh(webmercator, at_webmercator_bounds, heights, false);
    REQUIRE(mesh->vertices().size() > 0);
    CHECK(mesh->vertices().size() > 100);
    CHECK(mesh->vertices().size() < 200);
    const auto mesh_bb = mesh->bbox();
    CHECK(mesh_bb.min.x >= at_webmercator_bounds.getMinX() * 0.99999999999999);
    CHECK(mesh_bb.min.y >= at_webmercator_bounds.getMinY() * 0.99999999999999);
    CHECK(mesh_bb.min.z >= 50.0);
    CHECK(mesh_bb.max.x <= at_webmercator_bounds.getMaxX() * 1.00000000000001);
    CHECK(mesh_bb.max.y <= at_webmercator_bounds.getMaxY() * 1.00000000000001);
    CHECK(mesh_bb.max.z <= 4000.0);
  }

  SECTION("mesh vertex ranges wgs84") {
    const auto converter = cesium_tin_terra::TileWriter(ParallelTiler::Border::No);
    const auto at_wgs84_bounds = ctb::CRSBounds(9.5, 46.4, 17.1, 49.0);

    OGRSpatialReference wgs84;
    wgs84.importFromEPSG(4326);
    wgs84.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    const auto dataset = Dataset::make_shared(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");
    const auto reader = DatasetReader(dataset, wgs84, 1, false);
    const auto heights = reader.read(at_wgs84_bounds, 257, 257);

    const auto mesh = converter.toMesh(wgs84, at_wgs84_bounds, heights, false);
    REQUIRE(mesh->vertices().size() > 0);
    CHECK(mesh->vertices().size() > 100);
    CHECK(mesh->vertices().size() < 200);
    const auto mesh_bb = mesh->bbox();
    CHECK(mesh_bb.min.x >= at_wgs84_bounds.getMinX());
    CHECK(mesh_bb.min.y >= at_wgs84_bounds.getMinY());
    CHECK(mesh_bb.min.z >= 50.0);
    CHECK(mesh_bb.max.x <= at_wgs84_bounds.getMaxX());
    CHECK(mesh_bb.max.y <= at_wgs84_bounds.getMaxY());
    CHECK(mesh_bb.max.z <= 4000.0);
  }

  SECTION("mesh vertex ranges unit range scale") {
    const auto converter = cesium_tin_terra::TileWriter(ParallelTiler::Border::No);
    const auto at_wgs84_bounds = ctb::CRSBounds(9.5, 46.4, 17.1, 49.0);

    OGRSpatialReference wgs84;
    wgs84.importFromEPSG(4326);
    wgs84.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    const auto dataset = Dataset::make_shared(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");
    const auto reader = DatasetReader(dataset, wgs84, 1, false);
    const auto heights = reader.read(at_wgs84_bounds, 257, 257);

    const auto mesh = converter.toMesh(wgs84, at_wgs84_bounds, heights, true);
    REQUIRE(mesh->vertices().size() > 0);
    CHECK(mesh->vertices().size() > 100);
    CHECK(mesh->vertices().size() < 200);
    const auto mesh_bb = mesh->bbox();
    CHECK(mesh_bb.min.x >= 0);
    CHECK(mesh_bb.min.y >= 0);
    CHECK(mesh_bb.min.z >= 0);
    CHECK(mesh_bb.max.x <= 1);
    CHECK(mesh_bb.max.y <= 1);
    CHECK(mesh_bb.max.z <= 1);
  }


  SECTION("mesh vertex ranges webmercator global") {
    const auto converter = cesium_tin_terra::TileWriter(ParallelTiler::Border::No);
    const auto at_bounds = ctb::CRSBounds(-180, -80, 180, 80);
    OGRSpatialReference webmercator;
    webmercator.importFromEPSG(3857);
    webmercator.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    OGRSpatialReference wgs84;
    wgs84.importFromEPSG(4326);
    wgs84.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    const auto at_webmercator_bounds = srs::nonExactBoundsTransform(at_bounds, wgs84, webmercator);
    //    const auto at_webmercator_bounds = ctb::CRSBounds(1100000.0, 5900000.0, 1800000.0, 6200000.0);

    const auto dataset = Dataset::make_shared(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");
    const auto reader = DatasetReader(dataset, webmercator, 1, false);
    const auto heights = reader.read(at_webmercator_bounds, 257, 257);

    const auto mesh = converter.toMesh(webmercator, at_webmercator_bounds, heights, false, 17);
    REQUIRE(mesh->vertices().size() > 0);
    CHECK(mesh->vertices().size() == 289);
    const auto mesh_bb = mesh->bbox();
    CHECK(mesh_bb.min.x >= at_webmercator_bounds.getMinX());
    CHECK(mesh_bb.min.y >= at_webmercator_bounds.getMinY());
    CHECK(mesh_bb.min.z >= 0);
    CHECK(mesh_bb.max.x <= at_webmercator_bounds.getMaxX());
    CHECK(mesh_bb.max.y <= at_webmercator_bounds.getMaxY());
    CHECK(mesh_bb.max.z <= 4000.0);
  }

  SECTION("mesh vertex ranges wgs84 global") {
    const auto converter = cesium_tin_terra::TileWriter(ParallelTiler::Border::No);
    const auto at_wgs84_bounds = ctb::CRSBounds(-180, -90, 180, 90);

    OGRSpatialReference wgs84;
    wgs84.importFromEPSG(4326);
    wgs84.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    const auto dataset = Dataset::make_shared(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");
    const auto reader = DatasetReader(dataset, wgs84, 1, false);
    const auto heights = reader.read(at_wgs84_bounds, 257, 257);

    const auto mesh = converter.toMesh(wgs84, at_wgs84_bounds, heights, false, 17);
    REQUIRE(mesh->vertices().size() > 0);
    CHECK(mesh->vertices().size() == 289);
    const auto mesh_bb = mesh->bbox();
    CHECK(mesh_bb.min.x >= at_wgs84_bounds.getMinX());
    CHECK(mesh_bb.min.y >= at_wgs84_bounds.getMinY());
    CHECK(mesh_bb.min.z >= 0);
    CHECK(mesh_bb.max.x <= at_wgs84_bounds.getMaxX());
    CHECK(mesh_bb.max.y <= at_wgs84_bounds.getMaxY());
    CHECK(mesh_bb.max.z <= 4000.0);
  }

  SECTION("mesh vertex ranges unit range scale global") {
    const auto converter = cesium_tin_terra::TileWriter(ParallelTiler::Border::No);
    const auto at_wgs84_bounds = ctb::CRSBounds(-180, -90, 180, 90);

    OGRSpatialReference wgs84;
    wgs84.importFromEPSG(4326);
    wgs84.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    const auto dataset = Dataset::make_shared(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");
    const auto reader = DatasetReader(dataset, wgs84, 1, false);
    const auto heights = reader.read(at_wgs84_bounds, 257, 257);

    const auto mesh = converter.toMesh(wgs84, at_wgs84_bounds, heights, true, 17);
    REQUIRE(mesh->vertices().size() > 0);
    CHECK(mesh->vertices().size() == 289);
    const auto mesh_bb = mesh->bbox();
    CHECK(mesh_bb.min.x >= 0);
    CHECK(mesh_bb.min.y >= 0);
    CHECK(mesh_bb.min.z >= 0);
    CHECK(mesh_bb.max.x <= 1);
    CHECK(mesh_bb.max.y <= 1);
    CHECK(mesh_bb.max.z <= 1);
  }

  SECTION("mesh edges unit range scale global") {
    const auto converter = cesium_tin_terra::TileWriter(ParallelTiler::Border::No);
    const auto at_wgs84_bounds = ctb::CRSBounds(-180, -90, 180, 90);

    OGRSpatialReference wgs84;
    wgs84.importFromEPSG(4326);
    wgs84.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    const auto dataset = Dataset::make_shared(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");
    const auto reader = DatasetReader(dataset, wgs84, 1, false);
    const auto heights = reader.read(at_wgs84_bounds, 257, 257);

    for (unsigned nv : {2u, 3u, 5u, 9u}) {
      const auto mesh = converter.toMesh(wgs84, at_wgs84_bounds, heights, true, nv);
      CHECK(mesh->vertices().size() == nv * nv);
      const auto mesh_bb = mesh->bbox();
      CHECK(mesh_bb.min.x == 0);
      CHECK(mesh_bb.min.y == 0);
      CHECK(mesh_bb.min.z == 0);
      CHECK(mesh_bb.max.x == 1);
      CHECK(mesh_bb.max.y == 1);
      CHECK(mesh_bb.max.z == 0);

      unsigned nx0 = 0;
      unsigned ny0 = 0;
      unsigned nx1 = 0;
      unsigned ny1 = 0;
      for (const auto& v : mesh->vertices_as_vector()) {
        nx0 += std::abs(v.x - 0) < (1.0 / (1 << 16));
        ny0 += std::abs(v.y - 0) < (1.0 / (1 << 16));
        nx1 += std::abs(v.x - 1) < (1.0 / (1 << 16));
        ny1 += std::abs(v.y - 1) < (1.0 / (1 << 16));
      }
      CHECK(nx0 == nv);
      CHECK(ny0 == nv);
      CHECK(nx1 == nv);
      CHECK(ny1 == nv);
    }
  }

  SECTION("cesium terrain correct header") {
    auto generator = cesium_tin_terra::make_generator(ATB_TEST_DATA_DIR "/austria/at_mgi.tif", "./unittest_tiles/", ctb::Grid::Srs::SphericalMercator, ParallelTiler::Scheme::Tms, ParallelTiler::Border::No);
    generator.setWarnOnMissingOverviews(false);
    generator.process({4, 4});
    CHECK(std::filesystem::exists("./unittest_tiles/4/8/10.terrain"));
  }
}
