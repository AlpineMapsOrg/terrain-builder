#include <catch2/catch.hpp>
#include <opencv2/opencv.hpp>

#include "../catch2_helpers.h"
#include "mesh/io.h"
#include "convert.h"
#include "cgal.h"

TEST_CASE("convert rountrip keeps precision") {
    TerrainMesh mesh;

    const double pi = std::numbers::pi_v<double>;
    REQUIRE((double)(float)pi != pi);

    mesh.positions.push_back(glm::dvec3(0, 0, 0));
    mesh.positions.push_back(glm::dvec3(pi, 0, 0));
    mesh.positions.push_back(glm::dvec3(0, pi, 0));

    mesh.triangles.push_back(glm::uvec3(0, 2, 1));

    const SurfaceMesh cgal_mesh = convert::mesh2cgal(mesh);
    TerrainMesh roundtrip_mesh = convert::cgal2mesh(cgal_mesh);

    sort_and_normalize_triangles(mesh.triangles);
    sort_and_normalize_triangles(roundtrip_mesh.triangles);

    REQUIRE(roundtrip_mesh.positions == mesh.positions);
    REQUIRE(roundtrip_mesh.triangles == mesh.triangles);
}
