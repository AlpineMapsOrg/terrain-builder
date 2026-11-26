/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
 * Copyright (C) 2026 Martin Braunsperger
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

#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>

#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>

#include "mask.h"
#include "earth.h"
#include "gis/CplError.h"
#include "gis/srs/well_known.h"
#include "io/text.h"
#include "mesh/cgal.h"
#include "mesh/connectivity/manifold.h"
#include "mesh/convert.h"

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

TEST_CASE("masks split at the antimeridian are projected around their centre", "[sf_merger][mask]") {
    MultipolygonWithHoles2 polygons;
    polygons.add_polygon(make_square(glm::dvec2(170, -20), glm::dvec2(180, -15)));
    polygons.add_polygon(make_square(glm::dvec2(-180, -20), glm::dvec2(-170, -15)));
    const SpherePolygonMask mask = REQUIRE_VALUE(mask::project_onto_sphere(ReferencedPolygonMask { std::move(polygons), gis::srs::wgs84() }, earth::radius));
    REQUIRE(mask.polygons.number_of_polygons_with_holes() == 2);
    radix::geometry::Aabb2d projected_bounds;
    for (const auto& polygon : mask.polygons.polygons_with_holes()) {
        for (const auto& point : polygon.outer_boundary()) {
            const glm::dvec2 projected = convert::to_glm_point(point);
            REQUIRE(std::isfinite(projected.x));
            REQUIRE(std::isfinite(projected.y));
            projected_bounds.expand_by(projected);
        }
    }
    // In radians; the mask spans 20 by 5 degrees. A tangent point far from the mask would spread
    // it over the projection, or, at a pole, make it degenerate.
    CHECK(projected_bounds.width() < 0.5);
    CHECK(projected_bounds.height() < 0.5);
}

TEST_CASE("load_referenced_from_path fails for a missing mask", "[sf_merger][mask]") {
    const TempDirectory directory("mask_missing");

    const mask::LoadReferencedError error = REQUIRE_ERROR(mask::load_referenced_from_path(directory.path() / "missing.geojson"));

    CHECK(error.holds<gis::CplError>());
}

TEST_CASE("load_referenced_from_path fails for a mask of points", "[sf_merger][mask]") {
    const TempDirectory directory("mask_of_points");
    const std::filesystem::path path = directory.path() / "point.geojson";
    REQUIRE_VALUE(io::text::write(R"({"type": "FeatureCollection", "features": [{"type": "Feature", "properties": {}, "geometry": {"type": "Point", "coordinates": [11.0, 47.0]}}]})", path));

    const mask::LoadReferencedError error = REQUIRE_ERROR(mask::load_referenced_from_path(path));

    CHECK(error.holds_code(gis::vector_mask::InvalidMask::UnsupportedGeometry));
}
