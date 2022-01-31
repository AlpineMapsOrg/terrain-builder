#include <catch2/catch.hpp>

#include <glm/fwd.hpp>
#include <random>

#include "Image.h"
#include "cesium_tin_terra.h"
#include "Dataset.h"
#include "DatasetReader.h"
#include "srs.h"
#include "tntn/QuantizedMeshIO.h"
#include "tntn/MeshIO.h"
#include "tntn/terra_meshing.h"
#include "tntn/geometrix.h"

using namespace tntn::detail;

namespace tntn {
namespace unittests {

TEST_CASE("zig_zag_encode on reference values", "[tntn]")
{
    CHECK(zig_zag_encode(0) == 0);
    CHECK(zig_zag_encode(-1) == 1);
    CHECK(zig_zag_encode(1) == 2);
    CHECK(zig_zag_encode(-2) == 3);
    CHECK(zig_zag_encode(2) == 4);

    CHECK(zig_zag_encode(16383) == 16383 * 2);
    CHECK(zig_zag_encode(-16383) == 16383 * 2 - 1);

    CHECK(zig_zag_encode(16384) == 16384 * 2);
    CHECK(zig_zag_encode(-16384) == 16384 * 2 - 1);

    CHECK(zig_zag_encode(16385) == 16385 * 2);
    CHECK(zig_zag_encode(-16385) == 16385 * 2 - 1);

    CHECK(zig_zag_encode(32767) == 32767 * 2);
    CHECK(zig_zag_encode(-32767) == 32767 * 2 - 1);

    CHECK(zig_zag_encode(-32768) == 32768 * 2 - 1);
}

TEST_CASE("zig_zag_decode on reference values", "[tntn]")
{
    CHECK(zig_zag_decode(0) == 0);
    CHECK(zig_zag_decode(1) == -1);
    CHECK(zig_zag_decode(2) == 1);
    CHECK(zig_zag_decode(3) == -2);
    CHECK(zig_zag_decode(4) == 2);

    CHECK(zig_zag_decode(16383 * 2) == 16383);
    CHECK(zig_zag_decode(16383 * 2 - 1) == -16383);

    CHECK(zig_zag_decode(16384 * 2) == 16384);
    CHECK(zig_zag_decode(16384 * 2 - 1) == -16384);

    CHECK(zig_zag_decode(16385 * 2) == 16385);
    CHECK(zig_zag_decode(16385 * 2 - 1) == -16385);

    CHECK(zig_zag_decode(32767 * 2) == 32767);
    CHECK(zig_zag_decode(32767 * 2 - 1) == -32767);

    CHECK(zig_zag_decode(32768 * 2 - 1) == -32768);
}

TEST_CASE("zigzag encode diecode round trip", "[tntn]") {
  const std::vector<int16_t> input = {0, 32767, 3838, 0, 30033, 0, 1, 2, 3, 32767, 32766, 32765, 0, 32767};
  const auto encoded = quantized_mesh_encode(input);
  const auto decoded = quantized_mesh_decode(encoded);
  CHECK(decoded == input);
}

TEST_CASE("header is plausible with webmercator mesh", "[tntn]")
{
  // we are not checking exact here, because that would mean reimplementing the equations.
  // right now i don't really see the point, maybe i'll regret later ^^.
  // my hope is, that these things were implemented by ctb and tin-terrain correctly, and
  // i'm only checking for the correct transformation (ecef / wgs84 / webmercator)

  OGRSpatialReference ecef_srs;
  ecef_srs.importFromEPSG(4978);
  ecef_srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  // let's get a mesh of a part of austria (where we know the bounds etc)
  const auto converter = cesium_tin_terra::TileWriter(Tiler::Border::No);
  const auto at_wgs84_bounds = ctb::CRSBounds(11.362082472, 46.711274137, 12.631425730, 47.945935885);
  OGRSpatialReference webmercator;
  webmercator.importFromEPSG(3857);
  webmercator.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  OGRSpatialReference wgs84;
  wgs84.importFromEPSG(4326);
  wgs84.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  const auto check_header = [&](const QuantizedMeshHeader& header) {
    CHECK(header.MinimumHeight > 420);
    CHECK(header.MinimumHeight < 460);
    CHECK(header.MaximumHeight > 3480);
    CHECK(header.MaximumHeight < 3530);
    //  const auto box_a = glm::dvec3(4295537, 863174, 4620131); // at_bounds.min and min height with https://www.oc.nps.edu/oc2902w/coord/llhxyz.htm
    //  const auto box_b = glm::dvec3(4178876, 936496, 4715449); // at_bounds.max and max height with the same tool
    const auto box_min = glm::dvec3(4178876, 863174, 4620131); // min of the two above
    const auto box_max = glm::dvec3(4295537, 936496, 4715449); // max of the two above; note that this isn't a bounding box anymore, but it should be about right.
    CHECK(header.bounding_sphere_center.x > box_min.x);
    CHECK(header.bounding_sphere_center.y > box_min.y);
    CHECK(header.bounding_sphere_center.z > box_min.z);
    CHECK(header.bounding_sphere_center.x < box_max.x);
    CHECK(header.bounding_sphere_center.y < box_max.y);
    CHECK(header.bounding_sphere_center.z < box_max.z);
    CHECK(header.BoundingSphereRadius < glm::length(box_max - box_min));
    CHECK(header.BoundingSphereRadius > glm::length(box_max - box_min) / 4);
    CHECK(header.center.x > box_min.x);
    CHECK(header.center.y > box_min.y);
    CHECK(header.center.z > box_min.z);
    CHECK(header.center.x < box_max.x);
    CHECK(header.center.y < box_max.y);
    CHECK(header.center.z < box_max.z);

    CHECK(header.horizon_occlusion.x > 0);
    CHECK(header.horizon_occlusion.y > 0);
    CHECK(header.horizon_occlusion.z > 0);
    CHECK(header.horizon_occlusion.x < 1.1);
    CHECK(header.horizon_occlusion.y < 1.1);
    CHECK(header.horizon_occlusion.z < 1.1);
    // Constants taken from http://cesiumjs.org/2013/04/25/Horizon-culling
    constexpr double llh_ecef_radiusX = 6378137.0;
    constexpr double llh_ecef_radiusY = 6378137.0;
    constexpr double llh_ecef_radiusZ = 6356752.3142451793;
    const auto occlusion_point_in_wgs84 = srs::to(ecef_srs, wgs84, header.horizon_occlusion * glm::dvec3(llh_ecef_radiusX, llh_ecef_radiusY, llh_ecef_radiusZ));
    CHECK(at_wgs84_bounds.getMinX() < occlusion_point_in_wgs84.x);
    CHECK(at_wgs84_bounds.getMaxX() > occlusion_point_in_wgs84.x);
    CHECK(at_wgs84_bounds.getMinY() < occlusion_point_in_wgs84.y);
    CHECK(at_wgs84_bounds.getMaxY() > occlusion_point_in_wgs84.y);
    CHECK(occlusion_point_in_wgs84.z > header.MaximumHeight);
    CHECK(occlusion_point_in_wgs84.z < 10'000'000);
  };

  SECTION("webmercator") {
    const auto mesh_scale_0_to_1 = false;
    const auto at_webmercator_bounds = srs::nonExactBoundsTransform(at_wgs84_bounds, wgs84, webmercator);
    //    const auto at_webmercator_bounds = ctb::CRSBounds(1100000.0, 5900000.0, 1800000.0, 6200000.0);

    const auto dataset = Dataset::make_shared(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");
    const auto reader = DatasetReader(dataset, webmercator, 1);
    const auto heights = reader.read(at_webmercator_bounds, 512, 512);

    const auto mesh = converter.toMesh(webmercator, at_webmercator_bounds, heights, mesh_scale_0_to_1);
    const auto bbox = converter.computeBbox(at_webmercator_bounds, heights);
    QuantizedMeshHeader header = tntn::detail::quantised_mesh_header(*mesh, bbox, webmercator, mesh_scale_0_to_1);
    check_header(header);
  }

  SECTION("wgs84") {
    const auto mesh_scale_0_to_1 = false;
    const auto dataset = Dataset::make_shared(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");
    const auto reader = DatasetReader(dataset, wgs84, 1);
    const auto heights = reader.read(at_wgs84_bounds, 512, 512);

    const auto mesh = converter.toMesh(wgs84, at_wgs84_bounds, heights, mesh_scale_0_to_1);
    const auto bbox = converter.computeBbox(at_wgs84_bounds, heights);
    QuantizedMeshHeader header = tntn::detail::quantised_mesh_header(*mesh, bbox, wgs84, mesh_scale_0_to_1);
    check_header(header);
  }

  SECTION("webmercator, mesh scaled to unit cube") {
    const auto mesh_scale_0_to_1 = true;
    const auto at_webmercator_bounds = srs::nonExactBoundsTransform(at_wgs84_bounds, wgs84, webmercator);
    //    const auto at_webmercator_bounds = ctb::CRSBounds(1100000.0, 5900000.0, 1800000.0, 6200000.0);

    const auto dataset = Dataset::make_shared(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");
    const auto reader = DatasetReader(dataset, webmercator, 1);
    const auto heights = reader.read(at_webmercator_bounds, 512, 512);

    const auto mesh = converter.toMesh(webmercator, at_webmercator_bounds, heights, mesh_scale_0_to_1);
    const auto bbox = converter.computeBbox(at_webmercator_bounds, heights);
    QuantizedMeshHeader header = tntn::detail::quantised_mesh_header(*mesh, bbox, webmercator, mesh_scale_0_to_1);
    check_header(header);
  }

  SECTION("wgs84, mesh scaled to unit cube") {
    const auto mesh_scale_0_to_1 = true;
    const auto dataset = Dataset::make_shared(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");
    const auto reader = DatasetReader(dataset, wgs84, 1);
    const auto heights = reader.read(at_wgs84_bounds, 512, 512);

    const auto mesh = converter.toMesh(wgs84, at_wgs84_bounds, heights, mesh_scale_0_to_1);
    const auto bbox = converter.computeBbox(at_wgs84_bounds, heights);
    QuantizedMeshHeader header = tntn::detail::quantised_mesh_header(*mesh, bbox, wgs84, mesh_scale_0_to_1);
    check_header(header);
  }
}



TEST_CASE("header is plausible on all globe quarters", "[tntn]")
{
  // we are not checking exact here, because that would mean reimplementing the equations.
  // right now i don't really see the point, maybe i'll regret later ^^.
  // my hope is, that these things were implemented by ctb and tin-terrain correctly, and
  // i'm only checking for the correct transformation (ecef / wgs84 / webmercator)

  OGRSpatialReference ecef_srs;
  ecef_srs.importFromEPSG(4978);
  ecef_srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  // let's get a mesh of a part of austria (where we know the bounds etc)
  const auto converter = cesium_tin_terra::TileWriter(Tiler::Border::No);
  const std::array at_wgs84_bounds_array = {
                                            ctb::CRSBounds(0, 0, 90, 90),
                                            ctb::CRSBounds(0, -90, 90, 0),
                                            ctb::CRSBounds(90, 0, 180, 90),
                                            ctb::CRSBounds(90, -90, 180, 0),

                                            ctb::CRSBounds(-180, 0, -90, 90),
                                            ctb::CRSBounds(-180, -90, -90, 0),
                                            ctb::CRSBounds(-90, 0, 0, 90),
                                            ctb::CRSBounds(-90, -90, 0, 0)};
  OGRSpatialReference webmercator;
  webmercator.importFromEPSG(3857);
  webmercator.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  OGRSpatialReference wgs84;
  wgs84.importFromEPSG(4326);
  wgs84.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  for (const auto at_wgs84_bounds : at_wgs84_bounds_array) {
    const auto check_header = [&](const QuantizedMeshHeader& header) {
      CHECK(header.MinimumHeight >= 0);
      CHECK(header.MinimumHeight < 10);
      CHECK(header.MaximumHeight >= 0);
      CHECK(header.MaximumHeight < 4000);
      const auto [box_a, box_b] = srs::toECEF(wgs84, tntn::BBox3D(glm::dvec3(at_wgs84_bounds.getMinX(), at_wgs84_bounds.getMinY(), 0), glm::dvec3(at_wgs84_bounds.getMaxX(), at_wgs84_bounds.getMaxY(), 0)));
      const auto box_min = glm::dvec3(std::min(box_a.x, box_b.x) - 1.0, std::min(box_a.y, box_b.y) - 1.0, std::min(box_a.z, box_b.z) - 1.0);
      const auto box_max = glm::dvec3(std::max(box_a.x, box_b.x) + 1.0, std::max(box_a.y, box_b.y) + 1.0, std::max(box_a.z, box_b.z) + 1.0);
      CHECK(header.bounding_sphere_center.x > box_min.x);
      CHECK(header.bounding_sphere_center.y > box_min.y);
      CHECK(header.bounding_sphere_center.z > box_min.z);
      CHECK(header.bounding_sphere_center.x < box_max.x);
      CHECK(header.bounding_sphere_center.y < box_max.y);
      CHECK(header.bounding_sphere_center.z < box_max.z);
      CHECK(header.BoundingSphereRadius < glm::length(box_max - box_min));
      CHECK(header.BoundingSphereRadius > glm::length(box_max - box_min) / 4);
      CHECK(header.center.x > box_min.x);
      CHECK(header.center.y > box_min.y);
      CHECK(header.center.z > box_min.z);
      CHECK(header.center.x < box_max.x);
      CHECK(header.center.y < box_max.y);
      CHECK(header.center.z < box_max.z);

      CHECK(header.horizon_occlusion.x > -1.1);
      CHECK(header.horizon_occlusion.y > -1.1);
      CHECK(header.horizon_occlusion.z > -1.1);
      CHECK(header.horizon_occlusion.x < 1.1);
      CHECK(header.horizon_occlusion.y < 1.1);
      CHECK(header.horizon_occlusion.z < 1.1);
      // Constants taken from http://cesiumjs.org/2013/04/25/Horizon-culling
      constexpr double llh_ecef_radiusX = 6378137.0;
      constexpr double llh_ecef_radiusY = 6378137.0;
      constexpr double llh_ecef_radiusZ = 6356752.3142451793;
      const auto occlusion_point_in_wgs84 = srs::to(ecef_srs, wgs84, header.horizon_occlusion * glm::dvec3(llh_ecef_radiusX, llh_ecef_radiusY, llh_ecef_radiusZ));
      CHECK(at_wgs84_bounds.getMinX() < occlusion_point_in_wgs84.x);
      CHECK(at_wgs84_bounds.getMaxX() > occlusion_point_in_wgs84.x);
      CHECK(at_wgs84_bounds.getMinY() < occlusion_point_in_wgs84.y);
      CHECK(at_wgs84_bounds.getMaxY() > occlusion_point_in_wgs84.y);
      CHECK(occlusion_point_in_wgs84.z > header.MaximumHeight);
//      CHECK(occlusion_point_in_wgs84.z < 200'000'000);
    };

    // no section, because in loop
    /*SECTION("wgs84") */{
      const auto mesh_scale_0_to_1 = false;
      const auto heights = HeightData(256, 256);

      const auto mesh = converter.toMesh(wgs84, at_wgs84_bounds, heights, mesh_scale_0_to_1);
      const auto bbox = converter.computeBbox(at_wgs84_bounds, heights);
      QuantizedMeshHeader header = tntn::detail::quantised_mesh_header(*mesh, bbox, wgs84, mesh_scale_0_to_1);
      check_header(header);
    }

    /*SECTION("wgs84, mesh scaled to unit cube") */{
      const auto mesh_scale_0_to_1 = true;
      const auto heights = HeightData(256, 256);

      const auto mesh = converter.toMesh(wgs84, at_wgs84_bounds, heights, mesh_scale_0_to_1);
      const auto bbox = converter.computeBbox(at_wgs84_bounds, heights);
      QuantizedMeshHeader header = tntn::detail::quantised_mesh_header(*mesh, bbox, wgs84, mesh_scale_0_to_1);
      check_header(header);
    }
  }

}

TEST_CASE("edge vertices correct", "[tntn]") {
  OGRSpatialReference ecef_srs;
  ecef_srs.importFromEPSG(4978);
  ecef_srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  // let's get a mesh of a part of austria (where we know the bounds etc)
  const auto converter = cesium_tin_terra::TileWriter(Tiler::Border::No);
  const auto at_wgs84_bounds = ctb::CRSBounds(11.362082472, 46.711274137, 12.631425730, 47.945935885);
  OGRSpatialReference webmercator;
  webmercator.importFromEPSG(3857);
  webmercator.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  OGRSpatialReference wgs84;
  wgs84.importFromEPSG(4326);
  wgs84.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  const auto dataset = Dataset::make_shared(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");

  const auto run_test = [&](bool mesh_scale_0_to_1, const OGRSpatialReference& srs, const ctb::CRSBounds& wgs84_bounds, unsigned n_vertex_grid_cells, unsigned tile_grid_size) {
    const auto reader = DatasetReader(dataset, srs, 1, false);
    const auto srs_bounds = srs::nonExactBoundsTransform(wgs84_bounds, wgs84, srs);
    const auto heights = reader.read(srs_bounds, tile_grid_size + 1, tile_grid_size + 1);

    const auto mesh = converter.toMesh(srs, srs_bounds, heights, mesh_scale_0_to_1, n_vertex_grid_cells + 1);
    const auto correct_mesh_bounds = mesh_scale_0_to_1 ? ctb::CRSBounds(0.0, 0.0, 1.0, 1.0) : srs_bounds;

    CHECK(mesh->bbox().min.x == correct_mesh_bounds.getMinX());
    CHECK(mesh->bbox().min.y == correct_mesh_bounds.getMinY());
    CHECK(mesh->bbox().max.x == correct_mesh_bounds.getMaxX());
    CHECK(mesh->bbox().max.y == correct_mesh_bounds.getMaxY());

    unsigned nx0 = 0;
    unsigned ny0 = 0;
    unsigned nx1 = 0;
    unsigned ny1 = 0;
    for (const auto& v : mesh->vertices_as_vector()) {
      nx0 += std::abs(v.x - correct_mesh_bounds.getMinX()) / correct_mesh_bounds.getWidth() < (1.0 / (1 << 16));
      ny0 += std::abs(v.y - correct_mesh_bounds.getMinY()) / correct_mesh_bounds.getHeight() < (1.0 / (1 << 16));
      nx1 += std::abs(v.x - correct_mesh_bounds.getMaxX()) / correct_mesh_bounds.getWidth() < (1.0 / (1 << 16));
      ny1 += std::abs(v.y - correct_mesh_bounds.getMaxY()) / correct_mesh_bounds.getHeight() < (1.0 / (1 << 16));
    }
    CHECK(nx0 == n_vertex_grid_cells + 1);
    CHECK(ny0 == n_vertex_grid_cells + 1);
    CHECK(nx1 == n_vertex_grid_cells + 1);
    CHECK(ny1 == n_vertex_grid_cells + 1);

    const auto bbox = converter.computeBbox(srs_bounds, heights);
    tntn::detail::QuantizedMeshVertexData vdata = tntn::detail::quantised_mesh_vertex_data(*mesh, bbox, wgs84, mesh_scale_0_to_1);
    CHECK(vdata.northlings.size() == n_vertex_grid_cells + 1);
    CHECK(vdata.eastlings.size() == n_vertex_grid_cells + 1);
    CHECK(vdata.southlings.size() == n_vertex_grid_cells + 1);
    CHECK(vdata.westlings.size() == n_vertex_grid_cells + 1);

    const auto vs_decoded = tntn::detail::quantized_mesh_decode(vdata.vs);
    const auto us_decoded = tntn::detail::quantized_mesh_decode(vdata.us);

    for (const auto i : vdata.southlings) {
      CHECK(vs_decoded[i] == 0);
    }
    for (const auto i : vdata.westlings) {
      CHECK(us_decoded[i] == 0);
    }
    for (const auto i : vdata.northlings) {
      CHECK(vs_decoded[i] == 32767);
    }
    for (const auto i : vdata.eastlings) {
      CHECK(us_decoded[i] == 32767);
    }
  };


  SECTION("wgs84") {
    run_test(true, wgs84, at_wgs84_bounds, 16, 64);
    run_test(false, wgs84, at_wgs84_bounds, 16, 64);
    run_test(true, wgs84, at_wgs84_bounds, 16, 256);
    run_test(false, wgs84, at_wgs84_bounds, 16, 256);
    run_test(true, wgs84, at_wgs84_bounds, 16, 512);
    run_test(false, wgs84, at_wgs84_bounds, 16, 512);
  }
  SECTION("webmercator") {
    run_test(true, webmercator, at_wgs84_bounds, 16, 256);
    run_test(false, webmercator, at_wgs84_bounds, 16, 256);
  }
}

TEST_CASE("correctly stores indexed mesh", "[tntn]") {
  OGRSpatialReference ecef_srs;
  ecef_srs.importFromEPSG(4978);
  ecef_srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  // let's get a mesh of a part of austria (where we know the bounds etc)
  const auto converter = cesium_tin_terra::TileWriter(Tiler::Border::No);
  const auto at_wgs84_bounds = ctb::CRSBounds(11.362082472, 46.711274137, 12.631425730, 47.945935885);
  OGRSpatialReference webmercator;
  webmercator.importFromEPSG(3857);
  webmercator.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  OGRSpatialReference wgs84;
  wgs84.importFromEPSG(4326);
  wgs84.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  const auto dataset = Dataset::make_shared(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");

  const auto run_test = [&](bool mesh_scale_0_to_1, const OGRSpatialReference& srs, const ctb::CRSBounds& wgs84_bounds, unsigned n_vertex_grid_cells, unsigned tile_grid_size) {
    const auto reader = DatasetReader(dataset, srs, 1, false);
    const auto srs_bounds = srs::nonExactBoundsTransform(wgs84_bounds, wgs84, srs);
    const auto heights = reader.read(srs_bounds, tile_grid_size + 1, tile_grid_size + 1);

    const auto mesh = converter.toMesh(srs, srs_bounds, heights, mesh_scale_0_to_1, n_vertex_grid_cells + 1);
    CHECK(mesh->triangles().size() == n_vertex_grid_cells * n_vertex_grid_cells * 2);
    CHECK(mesh->vertices().size() == (n_vertex_grid_cells + 1) * (n_vertex_grid_cells + 1));

    const auto bbox = converter.computeBbox(srs_bounds, heights);
    tntn::detail::QuantizedMeshVertexData vdata = tntn::detail::quantised_mesh_vertex_data(*mesh, bbox, wgs84, mesh_scale_0_to_1);
    CHECK(vdata.us.size() == mesh->vertices().size());
    CHECK(vdata.vs.size() == mesh->vertices().size());
    CHECK(vdata.hs.size() == mesh->vertices().size());

  };


  SECTION("wgs84") {
    run_test(true, wgs84, at_wgs84_bounds, 4, 64);
    run_test(false, wgs84, at_wgs84_bounds, 4, 64);
    run_test(true, wgs84, at_wgs84_bounds, 4, 256);
    run_test(false, wgs84, at_wgs84_bounds, 4, 256);
    run_test(true, wgs84, at_wgs84_bounds, 4, 512);
    run_test(false, wgs84, at_wgs84_bounds, 4, 512);
  }
  SECTION("webmercator") {
    run_test(true, webmercator, at_wgs84_bounds, 4, 256);
    run_test(false, webmercator, at_wgs84_bounds, 4, 256);
  }
}

#if 1
TEST_CASE("quantized mesh writer/loader round trip on small mesh", "[tntn]")
{
    const int xscale = 2;
    const int yscale = 1;
    const double scale = (xscale + yscale) / 2.0;

    auto terrain_fn = [=](int x, int y) -> double {
        return 10 * scale * sin(x * 0.1 / scale) * sin(y * 0.1 / scale);
    };

    const int w = 100 * xscale;
    const int h = 100 * yscale;

    std::vector<Vertex> vertices;
    vertices.reserve(h * w);
    std::mt19937 generator(42); //fixed seed
    std::uniform_int_distribution<size_t> dist(0, 16);

    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            if(dist(generator) == 0)
            {
                vertices.push_back({x, y, terrain_fn(x, y)});
            }
        }
    }

    auto sp = std::make_unique<SurfacePoints>();
    sp->load_from_memory(std::move(vertices));
    auto mesh = generate_tin_terra(std::move(sp), 0.1);
    REQUIRE(mesh != nullptr);
    //CHECK(mesh->check_tin_properties());

    const auto original_faces_count = mesh->faces().size();

    mesh->generate_triangles();
    CHECK(mesh->triangles().size() == original_faces_count);

    auto mf = std::make_shared<MemoryFile>();
    //write_mesh_as_obj("terrain.obj", *mesh);
    //write_mesh_as_qm("terrain.terrain", *mesh);
    CHECK(write_mesh_as_qm(mf, *mesh));

    auto loaded_mesh = load_mesh_from_qm(mf);
    REQUIRE(loaded_mesh != nullptr);
    CHECK(loaded_mesh->faces().size() == original_faces_count);

    //TODO: investigate why this doesn't hold. Probably because of quantization differences?
    //CHECK(loaded_mesh->semantic_equal(*mesh));

    //write_mesh_as_obj("terrain.terrain.obj", *mesh);
}
#endif

#if 1
TEST_CASE("quantized mesh writer/loader round trip on empty mesh", "[tntn]")
{
    auto mesh = std::make_shared<Mesh>();

    auto mf = std::make_shared<MemoryFile>();

    CHECK_THROWS(write_mesh_as_qm(mf, *mesh));
    auto loaded_mesh = load_mesh_from_qm(mf);
    REQUIRE(loaded_mesh == nullptr);
    // CHECK(loaded_mesh->semantic_equal(*mesh));
}
#endif

#if 0
TEST_CASE("load reference quantized mesh", "[tntn]")
{
    auto loaded_mesh = load_mesh_from_qm("/Users/dietrich/Downloads/no-extensions.terrain");
    REQUIRE(loaded_mesh != nullptr);
    loaded_mesh->generate_triangles();
    REQUIRE(loaded_mesh->check_tin_properties());

    write_mesh_as_obj("no-extensions.obj", *loaded_mesh);
    write_mesh_as_qm("no-extensions.rewrite.terrain", *loaded_mesh);
    auto reloaded_mesh = load_mesh_from_qm("no-extensions.rewrite.terrain");
    REQUIRE(reloaded_mesh != nullptr);
    write_mesh_as_obj("no-extensions.rewrite.terrain.obj", *reloaded_mesh);
    REQUIRE(reloaded_mesh->check_tin_properties());

    CHECK(loaded_mesh->semantic_equal(*reloaded_mesh));
}
#endif

} // namespace unittests
} // namespace tntn
