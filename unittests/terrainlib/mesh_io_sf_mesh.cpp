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

#include <catch2/catch_test_macros.hpp>

#include <glm/glm.hpp>
#include <opencv2/core.hpp>

#include "../catch2_helpers.h"
#include "../opencv_helpers.h"
#include "../result_helpers.h"
#include "io/envelope.h"
#include "io/image/Format.h"
#include "mesh/SimpleMesh.h"
#include "mesh/io/sf_mesh.h"

TEST_CASE("validate refuses vertex data without vertices", "[mesh][io][sf_mesh]") {
    const mesh::io::sf_mesh::Payload payload{.vertex_count = 0, .face_count = 0, .triangles = {}, .positions = {1}, .uvs = {}, .texture = {}};

    const auto error = REQUIRE_ERROR(mesh::io::sf_mesh::validate(payload));

    CHECK(error.holds<mesh::io::sf_mesh::CorruptData>());
}

TEST_CASE("validate refuses triangle data without faces", "[mesh][io][sf_mesh]") {
    const mesh::io::sf_mesh::Payload payload{.vertex_count = 0, .face_count = 0, .triangles = {1}, .positions = {}, .uvs = {}, .texture = {}};

    const auto error = REQUIRE_ERROR(mesh::io::sf_mesh::validate(payload));

    CHECK(error.holds<mesh::io::sf_mesh::CorruptData>());
}

TEST_CASE("deserialize reads back a serialized mesh", "[mesh][io][sf_mesh]") {
    mesh::Simple mesh;
    mesh.positions = {glm::dvec3(0.1, 0, 0), glm::dvec3(1, 0.2, 0), glm::dvec3(0, 1, 0.3), glm::dvec3(1, 1, 0)};
    mesh.triangles = {glm::uvec3(0, 2, 1), glm::uvec3(1, 2, 3)};
    mesh.uvs = {glm::dvec2(0, 0), glm::dvec2(1, 0), glm::dvec2(0, 1), glm::dvec2(1, 1)};
    cv::Mat3b texture(2, 2);
    texture(0, 0) = cv::Vec3b(10, 20, 30);
    texture(0, 1) = cv::Vec3b(40, 50, 60);
    texture(1, 0) = cv::Vec3b(70, 80, 90);
    texture(1, 1) = cv::Vec3b(100, 110, 120);
    mesh.texture = texture;

    const auto bytes = REQUIRE_VALUE(mesh::io::sf_mesh::serialize(mesh, {.texture_format = io::image::Format::Png}));

    const mesh::Simple roundtrip_mesh = REQUIRE_VALUE(mesh::io::sf_mesh::deserialize(bytes));
    CHECK(roundtrip_mesh.positions == mesh.positions);
    CHECK(roundtrip_mesh.triangles == mesh.triangles);
    CHECK(roundtrip_mesh.uvs == mesh.uvs);
    REQUIRE(roundtrip_mesh.texture.has_value());
    CHECK(mat_equals(*roundtrip_mesh.texture, *mesh.texture));
}

TEST_CASE("deserialize refuses a payload that validate refuses", "[mesh][io][sf_mesh]") {
    const mesh::io::sf_mesh::Payload payload{.vertex_count = 0, .face_count = 0, .triangles = {}, .positions = {1}, .uvs = {}, .texture = {}};
    const auto bytes = REQUIRE_VALUE(io::envelope::serialize<mesh::io::sf_mesh::Schema>(payload));

    const auto error = REQUIRE_ERROR(mesh::io::sf_mesh::deserialize(bytes));

    CHECK(error.holds<mesh::io::sf_mesh::CorruptData>());
}
