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
#include <glm/glm.hpp>

#include "../result_helpers.h"
#include "mesh/SimpleMesh.h"
#include "mesh/validate/result.h"

TEST_CASE("validate accepts a valid mesh", "[mesh][validate]") {
    const mesh::Simple mesh({ { 0, 1, 2 } }, { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 } }, { { 0.0, 0.0 }, { 1.0, 0.0 }, { 0.0, 1.0 } });

    REQUIRE_VALUE(mesh::validate(mesh));
    REQUIRE_VALUE(mesh::validate_manifold(mesh));
}

TEST_CASE("validate_basic rejects inconsistent triangles and UVs", "[mesh][validate]") {
    std::vector<glm::uvec3> triangles = { { 0, 1, 2 } };
    const std::vector<glm::dvec3> positions = { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 } };
    std::vector<glm::dvec2> uvs = { { 0.0, 0.0 }, { 1.0, 0.0 }, { 0.0, 1.0 } };
    SECTION("not one UV per position") {
        uvs = { { 0.0, 0.0 }, { 1.0, 0.0 } };
    }
    SECTION("UV below 0") {
        uvs[1] = { -0.1, 0.0 };
    }
    SECTION("UV above 1") {
        uvs[2] = { 0.0, 1.1 };
    }
    SECTION("triangle names no vertex") {
        triangles = { { 0, 1, 3 } };
    }
    SECTION("degenerate triangle") {
        triangles = { { 0, 1, 1 } };
    }
    const mesh::Simple mesh(triangles, positions, uvs);

    const auto error = REQUIRE_ERROR(mesh::validate_basic(mesh));

    CHECK(error.holds<mesh::InvalidMesh>());
}

TEST_CASE("validate_unconnected rejects isolated vertices that validate_basic accepts", "[mesh][validate]") {
    const mesh::Simple mesh({ { 0, 1, 2 } }, { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 1.0, 1.0, 0.0 } });

    REQUIRE_VALUE(mesh::validate_basic(mesh));
    const auto error = REQUIRE_ERROR(mesh::validate_unconnected(mesh));

    CHECK(error.holds<mesh::InvalidMesh>());
}

TEST_CASE("validate_connected rejects several components that validate_unconnected accepts", "[mesh][validate]") {
    const mesh::Simple mesh(
        { { 0, 1, 2 }, { 3, 4, 5 } },
        { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 5.0, 0.0, 0.0 }, { 6.0, 0.0, 0.0 }, { 5.0, 1.0, 0.0 } });

    REQUIRE_VALUE(mesh::validate_unconnected(mesh));
    const auto error = REQUIRE_ERROR(mesh::validate_connected(mesh));

    CHECK(error.holds<mesh::InvalidMesh>());
}

TEST_CASE("validate rejects a non-manifold edge that validate_unconnected accepts", "[mesh][validate]") {
    // Three triangles share the edge from vertex 0 to vertex 1.
    const mesh::Simple mesh(
        { { 0, 1, 2 }, { 1, 0, 3 }, { 0, 1, 4 } },
        { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, -1.0, 0.0 }, { 0.0, 0.0, 1.0 } });

    REQUIRE_VALUE(mesh::validate_unconnected(mesh));
    const auto validate_error = REQUIRE_ERROR(mesh::validate(mesh));
    const auto manifold_error = REQUIRE_ERROR(mesh::validate_manifold(mesh));

    CHECK(validate_error.holds<mesh::InvalidMesh>());
    CHECK(manifold_error.holds<mesh::InvalidMesh>());
}
