/*****************************************************************************
 * AlpineMaps.org
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

#include <catch2/catch_test_macros.hpp>

#include <filesystem>

#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>

#include "utils.h"
#include "earth.h"
#include "geometry/geometry.h"
#include "mask.h"
#include "mesh/cgal.h"
#include "mesh/connectivity/manifold.h"
#include "mesh/convert.h"
#include "mesh/geometry.h"
#include "mesh/primitives.h"
#include "octree/Id.h"
#include "octree/Space.h"

namespace {

void check_closed_volume(const SimpleMesh &mesh) {
    const cgal::Mesh cgal_mesh = convert::to_cgal_mesh(mesh);
    CHECK(mesh::is_manifold(mesh));
    CHECK(CGAL::is_closed(cgal_mesh));
    CHECK(CGAL::is_valid_polygon_mesh(cgal_mesh));
    CHECK_FALSE(CGAL::Polygon_mesh_processing::does_self_intersect(cgal_mesh));
    CHECK(CGAL::Polygon_mesh_processing::does_bound_a_volume(cgal_mesh));
}

} // namespace

TEST_CASE("clip_on_mask uses union semantics", "[sf_merger][utils]") {
    const SimpleMesh source = mesh::make_rect(radix::geometry::Aabb<2, double>{{-3.0, -1.0}, {3.0, 1.0}});
    const MeshMask mask{
        .components = {
            mesh::make_box(radix::geometry::Aabb<3, double>{{-2.0, -2.0, -1.0}, {-1.0, 2.0, 1.0}}),
            mesh::make_box(radix::geometry::Aabb<3, double>{{1.0, -2.0, -1.0}, {2.0, 2.0, 1.0}}),
        }};

    const Cow<const SimpleMesh> inside = clip_on_mask(source, mask, true);
    CHECK_THAT(mesh::compute_surface_area(inside.get()), WithinEpsilon(4.0));

    const Cow<const SimpleMesh> outside = clip_on_mask(source, mask, false);
    CHECK_THAT(mesh::compute_surface_area(outside.get()), WithinEpsilon(8.0));
}

TEST_CASE("clip_on_mask without components uses empty-union semantics", "[sf_merger][utils]") {
    const SimpleMesh source = mesh::make_rect(radix::geometry::Aabb<2, double>{{-3.0, -1.0}, {3.0, 1.0}});
    const MeshMask mask;

    const Cow<const SimpleMesh> inside = clip_on_mask(source, mask, true);
    CHECK_FALSE(inside.is_ref());
    CHECK(inside->is_empty());

    const Cow<const SimpleMesh> outside = clip_on_mask(source, mask, false);
    CHECK(outside.is_ref());
    CHECK_FALSE(outside->is_empty());
}

TEST_CASE("clip_mask_on_bounds keeps the Tirol mask components valid", "[sf_merger][utils]") {
    const std::filesystem::path path = std::filesystem::path(ALP_TEST_DATA_DIR) / "sf_builder_merge_border/mask/tirol.shp";
    const glm::dvec2 radius_range = mask::pad_radius_range(earth::radius_range, 2.0);
    const MeshMask loaded = REQUIRE_VALUE(mask::load_from_path(path, radius_range));
    REQUIRE(loaded.components.size() == 3);
    for (const SimpleMesh &component : loaded.components) {
        check_closed_volume(component);
    }

    const octree::Id regression_node(15, 26290, 18610, 27235);
    const radix::geometry::Aabb3d bounds = geometry::pad_bounds_relative(octree::Space::earth().get_node_bounds(regression_node), 0.05);

    const MeshMask clipped = clip_mask_on_bounds(loaded, bounds);

    REQUIRE_FALSE(clipped.is_empty());
    for (const SimpleMesh &component : clipped.components) {
        check_closed_volume(component);
    }
}
