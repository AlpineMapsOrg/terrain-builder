/*****************************************************************************
 * Alpine Terrain Builder
 * Copyright (C) 2022 Adam Celarek <last name at cg tuwien ac at>
 * Copyright (C) 2022 alpinemaps.org
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

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <CGAL/Unique_hash_map.h>

#include "Dataset.h"
#include "ctb/GlobalMercator.hpp"
#include "ctb/Grid.hpp"
#include "srs.h"
#include "../catch2_helpers.h"
#include <fmt/core.h>

#include "terrain_mesh.h"
#include "mesh_builder.h"
#include "gltf_writer.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point3;
typedef CGAL::Surface_mesh<Point3> SurfaceMesh;
typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor VertexDescriptor;
typedef boost::graph_traits<SurfaceMesh>::edge_descriptor EdgeDescriptor;
typedef boost::graph_traits<SurfaceMesh>::face_descriptor FaceDescriptor;

Point3 glm2cgal(glm::dvec3 point) {
    return Point3(point[0], point[1], point[2]);
}
SurfaceMesh mesh2cgal(const TerrainMesh &mesh) {
    SurfaceMesh cgal_mesh;

    for (const glm::dvec3 &position : mesh.positions) {
        const CGAL::SM_Vertex_index vertex = cgal_mesh.add_vertex(glm2cgal(position));
        REQUIRE(vertex != SurfaceMesh::null_vertex());
    }

    for (const glm::uvec3 &triangle : mesh.triangles) {
        const CGAL::SM_Face_index face = cgal_mesh.add_face(
            CGAL::SM_Vertex_index(triangle.x),
            CGAL::SM_Vertex_index(triangle.y),
            CGAL::SM_Vertex_index(triangle.z));

        REQUIRE(face != SurfaceMesh::null_face());
    }

    return cgal_mesh;
}

size_t count_connected_components(const SurfaceMesh& mesh) {
    typedef CGAL::Unique_hash_map<FaceDescriptor, size_t> CcMap;
    typedef boost::associative_property_map<CcMap> CcPropertyMap;

    CcMap cc_map;
    CcPropertyMap cc_pmap(cc_map);
    const size_t num = CGAL::Polygon_mesh_processing::connected_components(mesh, cc_pmap);
    return num;
}

TEST_CASE("can build reference mesh tiles", "[terrainbuilder]") {
    std::vector<std::tuple<std::string, std::string, tile::Id>> test_cases = {
        {"tiny tile", "/austria/pizbuin_1m_epsg4326.tif", tile::Id(23, glm::uvec2(4430412, 2955980), tile::Scheme::SlippyMap)},
        {"small tile", "/austria/pizbuin_1m_epsg3857.tif", tile::Id(20, glm::uvec2(553801, 369497), tile::Scheme::SlippyMap)},
        {"tile on the border", "/austria/pizbuin_1m_epsg4326.tif", tile::Id(12, glm::uvec2(2162, 2652), tile::Scheme::Tms)},
        {"tile slightly larger than dataset", "/austria/pizbuin_1m_mgi.tif", tile::Id(11, glm::uvec2(1081, 721), tile::Scheme::SlippyMap)},
        {"huge tile", "/austria/pizbuin_1m_epsg3857.tif", tile::Id(6, glm::uvec2(33, 41), tile::Scheme::Tms)},
        {"giant tile", "/austria/at_mgi.tif", tile::Id(1, glm::uvec2(1, 0), tile::Scheme::SlippyMap)},
    };

    for (const auto &test : test_cases) {
        const auto [test_name, dataset_suffix, target_tile] = test;

        DYNAMIC_SECTION(test_name) {
            const std::filesystem::path dataset_path = std::filesystem::path(ATB_TEST_DATA_DIR).concat(dataset_suffix);
            Dataset dataset(dataset_path);

            OGRSpatialReference webmercator_srs;
            webmercator_srs.importFromEPSG(3857);
            webmercator_srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

            const ctb::Grid grid = ctb::GlobalMercator();
            const tile::SrsBounds tile_bounds = grid.srsBounds(target_tile, false);

            tile::SrsBounds output_tile_bounds(tile_bounds);
            tile::SrsBounds output_texture_bounds(tile_bounds);
            const TerrainMesh mesh = build_reference_mesh_tile(
                dataset,
                webmercator_srs,
                webmercator_srs, output_tile_bounds,
                webmercator_srs, output_texture_bounds,
                Border(0, 1, 1, 0),
                true);

            REQUIRE(mesh.positions.size() > 0);
            REQUIRE(mesh.uvs.size() == mesh.positions.size());
            REQUIRE(mesh.triangles.size() > 0);

            for (const glm::dvec2 uv : mesh.uvs) {
                REQUIRE(glm::all(glm::greaterThanEqual(uv, glm::dvec2(0))));
                REQUIRE(glm::all(glm::lessThanEqual(uv, glm::dvec2(1))));
            }

            const SurfaceMesh cgal_mesh = mesh2cgal(mesh);
            REQUIRE(cgal_mesh.is_valid(true));
            REQUIRE(CGAL::is_triangle_mesh(cgal_mesh));
            REQUIRE(count_connected_components(cgal_mesh) == 1);
            REQUIRE(!CGAL::Polygon_mesh_processing::does_self_intersect(cgal_mesh));
        }
    }
}
