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

#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_vector.hpp>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <glm/glm.hpp>
#include <radix/geometry.h>

#include "../catch2_helpers.h"
#include "mesh/SimpleMesh.h"
#include "mesh/TriangleSoup.h"
#include "mesh/cgal.h"
#include "mesh/convert.h"
#include "mesh/primitives.h"

TEST_CASE("make_box builds a closed box wound outward on the corners of its bounds", "[mesh][primitives]") {
    const radix::geometry::Aabb<3, double> bounds{{1.0, 2.0, 3.0}, {4.0, 6.0, 8.0}};

    const mesh::Simple box = mesh::make_box(bounds);

    const std::vector<glm::dvec3> corners = {
        {1.0, 2.0, 3.0}, {4.0, 2.0, 3.0}, {4.0, 6.0, 3.0}, {1.0, 6.0, 3.0},
        {1.0, 2.0, 8.0}, {4.0, 2.0, 8.0}, {4.0, 6.0, 8.0}, {1.0, 6.0, 8.0},
    };
    CHECK_THAT(box.positions, Catch::Matchers::UnorderedEquals(corners));
    const cgal::Mesh cgal_mesh = convert::to_cgal_mesh(box);
    REQUIRE(CGAL::is_closed(cgal_mesh));
    CHECK(CGAL::Polygon_mesh_processing::is_outward_oriented(cgal_mesh));
    CHECK_THAT(CGAL::to_double(CGAL::Polygon_mesh_processing::volume(cgal_mesh)), WithinEpsilon(60.0));
}

// The rectangle can be split along either diagonal, so both splits are accepted.
TEST_CASE("make_rect builds two triangles facing +z on the corners of its bounds", "[mesh][primitives]") {
    const radix::geometry::Aabb<2, double> bounds{{1.0, 2.0}, {4.0, 6.0}};

    const mesh::Simple rect = mesh::make_rect(bounds);

    const mesh::Simple split_from_min({{0, 1, 2}, {0, 2, 3}}, {{1.0, 2.0, 0.0}, {4.0, 2.0, 0.0}, {4.0, 6.0, 0.0}, {1.0, 6.0, 0.0}});
    const mesh::Simple split_from_max_x({{0, 1, 3}, {1, 2, 3}}, {{1.0, 2.0, 0.0}, {4.0, 2.0, 0.0}, {4.0, 6.0, 0.0}, {1.0, 6.0, 0.0}});
    const TriangleSoup soup = to_sorted_triangle_soup(rect);
    const bool is_a_split = soup == to_sorted_triangle_soup(split_from_min) || soup == to_sorted_triangle_soup(split_from_max_x);
    CHECK(is_a_split);
}

TEST_CASE("the primitives built around a centre equal those built from their bounds", "[mesh][primitives]") {
    SECTION("box") {
        const mesh::Simple box = mesh::make_box(glm::dvec3(2.5, 4.0, 5.5), glm::dvec3(3.0, 4.0, 5.0));
        const mesh::Simple expected = mesh::make_box(radix::geometry::Aabb<3, double>{{1.0, 2.0, 3.0}, {4.0, 6.0, 8.0}});
        CHECK(to_sorted_triangle_soup(box) == to_sorted_triangle_soup(expected));
    }

    SECTION("cube") {
        const mesh::Simple cube = mesh::make_cube(glm::dvec3(2.0, 3.0, 4.0), 2.0);
        const mesh::Simple expected = mesh::make_box(radix::geometry::Aabb<3, double>{{1.0, 2.0, 3.0}, {3.0, 4.0, 5.0}});
        CHECK(to_sorted_triangle_soup(cube) == to_sorted_triangle_soup(expected));
    }

    SECTION("rect") {
        const mesh::Simple rect = mesh::make_rect(glm::dvec2(2.5, 4.0), glm::dvec2(3.0, 4.0));
        const mesh::Simple expected = mesh::make_rect(radix::geometry::Aabb<2, double>{{1.0, 2.0}, {4.0, 6.0}});
        CHECK(to_sorted_triangle_soup(rect) == to_sorted_triangle_soup(expected));
    }

    SECTION("square") {
        const mesh::Simple square = mesh::make_square(glm::dvec2(2.0, 3.0), 2.0);
        const mesh::Simple expected = mesh::make_rect(radix::geometry::Aabb<2, double>{{1.0, 2.0}, {3.0, 4.0}});
        CHECK(to_sorted_triangle_soup(square) == to_sorted_triangle_soup(expected));
    }
}
