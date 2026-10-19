#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>

#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>

#include "mask.h"
#include "gdal/CplError.h"
#include "io/text.h"
#include "mesh/cgal.h"
#include "mesh/connectivity/manifold.h"
#include "mesh/convert.h"
#include "srs/System.h"

namespace {

Polygon2 make_square(const glm::dvec2 min, const glm::dvec2 max) {
    Polygon2 polygon;
    polygon.push_back(Point2(min.x, min.y));
    polygon.push_back(Point2(max.x, min.y));
    polygon.push_back(Point2(max.x, max.y));
    polygon.push_back(Point2(min.x, max.y));
    return polygon;
}

void check_closed_volume(const SimpleMesh &mesh) {
    const cgal::Mesh cgal_mesh = convert::to_cgal_mesh(mesh);
    CHECK(mesh::is_manifold(mesh));
    CHECK(CGAL::is_closed(cgal_mesh));
    CHECK(CGAL::is_valid_polygon_mesh(cgal_mesh));
    CHECK_FALSE(CGAL::Polygon_mesh_processing::does_self_intersect(cgal_mesh));
    CHECK(CGAL::Polygon_mesh_processing::does_bound_a_volume(cgal_mesh));
}

} // namespace

TEST_CASE("simplification_tolerance is a fixed tenth of a metre", "[sf_merger][mask]") {
    SECTION("projected") {
        const std::optional<double> tolerance = mask::simplification_tolerance(REQUIRE_VALUE(srs::from_epsg(31287)));
        REQUIRE(tolerance);
        CHECK_THAT(*tolerance, WithinEpsilon(0.1));
    }
    SECTION("geographic") {
        const srs::System geographic = srs::wgs84();
        const std::optional<double> tolerance = mask::simplification_tolerance(geographic);
        REQUIRE(tolerance);
        CHECK_THAT(*tolerance, WithinEpsilon(0.1 / geographic.GetSemiMajor() / geographic.GetAngularUnits()));
    }
}

TEST_CASE("simplify_geometry preserves polygon topology", "[sf_merger][mask]") {
    OGRLinearRing outer;
    outer.addPoint(0.0, 0.0);
    outer.addPoint(1.0, 0.01);
    outer.addPoint(2.0, 0.0);
    outer.addPoint(2.0, 2.0);
    outer.addPoint(0.0, 2.0);
    outer.addPoint(0.0, 0.0);

    OGRLinearRing hole;
    hole.addPoint(0.5, 0.5);
    hole.addPoint(1.0, 0.51);
    hole.addPoint(1.5, 0.5);
    hole.addPoint(1.5, 1.5);
    hole.addPoint(0.5, 1.5);
    hole.addPoint(0.5, 0.5);

    OGRPolygon polygon;
    REQUIRE(polygon.addRing(&outer) == OGRERR_NONE);
    REQUIRE(polygon.addRing(&hole) == OGRERR_NONE);
    REQUIRE(polygon.IsValid());

    const std::unique_ptr<OGRGeometry> simplified = mask::simplify_geometry(polygon, 0.1);

    REQUIRE(simplified);
    CHECK(simplified->exportToWkt() == "POLYGON ((0 0,2 0,2 2,0 2,0 0),(0.5 0.5,1.5 0.5,1.5 1.5,0.5 1.5,0.5 0.5))");
}

TEST_CASE("triangulate and extrude preserve polygon components", "[sf_merger][mask]") {
    const SphereProjector projector(glm::dvec3(100.0, 0.0, 0.0));
    glm::dvec2 second_min;
    SECTION("apart") {
        second_min = glm::dvec2(0.02, 0.02);
    }
    SECTION("touching at a point") {
        second_min = glm::dvec2(-0.02, 0.02);
    }
    MultipolygonWithHoles2 polygons;
    polygons.add_polygon(make_square(glm::dvec2(-0.04, -0.02), glm::dvec2(-0.02, 0.02)));
    polygons.add_polygon(make_square(second_min, glm::dvec2(0.04, 0.06)));

    const SphereMeshMask triangulated = mask::triangulate(SpherePolygonMask{std::move(polygons), projector});

    const std::vector<glm::dvec3> first_corners = {
        projector.unproject_point(glm::dvec2(-0.04, -0.02)),
        projector.unproject_point(glm::dvec2(-0.02, -0.02)),
        projector.unproject_point(glm::dvec2(-0.02, 0.02)),
        projector.unproject_point(glm::dvec2(-0.04, 0.02)),
    };
    const std::vector<glm::dvec3> second_corners = {
        projector.unproject_point(second_min),
        projector.unproject_point(glm::dvec2(0.04, second_min.y)),
        projector.unproject_point(glm::dvec2(0.04, 0.06)),
        projector.unproject_point(glm::dvec2(second_min.x, 0.06)),
    };
    REQUIRE(triangulated.components.size() == 2);
    CHECK_THAT(triangulated.components[0].positions, UnorderedWithinEpsilon(first_corners) || UnorderedWithinEpsilon(second_corners));
    CHECK_THAT(triangulated.components[1].positions, UnorderedWithinEpsilon(first_corners) || UnorderedWithinEpsilon(second_corners));

    const MeshMask extruded = mask::extrude(triangulated, glm::dvec2(90.0, 110.0));

    REQUIRE(extruded.components.size() == 2);
    check_closed_volume(extruded.components[0]);
    check_closed_volume(extruded.components[1]);
}

TEST_CASE("load_referenced_from_path fails for a missing mask", "[sf_merger][mask]") {
    const TempDirectory directory("mask_missing");

    const mask::LoadError error = REQUIRE_ERROR(mask::load_referenced_from_path(directory.path() / "missing.geojson"));

    CHECK(error.holds<gdal::CplError>());
}

TEST_CASE("load_referenced_from_path fails for a mask without polygons", "[sf_merger][mask]") {
    const TempDirectory directory("mask_without_polygons");
    const std::filesystem::path path = directory.path() / "point.geojson";
    REQUIRE_VALUE(io::write_text_to_path(R"({"type": "FeatureCollection", "features": [{"type": "Feature", "properties": {}, "geometry": {"type": "Point", "coordinates": [11.0, 47.0]}}]})", path));

    const mask::LoadError error = REQUIRE_ERROR(mask::load_referenced_from_path(path));

    CHECK(error.holds(mask::MaskError::NoPolygons));
}
