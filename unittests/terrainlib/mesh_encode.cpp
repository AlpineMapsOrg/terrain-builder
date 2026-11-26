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

#include <string_view>

#include <glm/glm.hpp>
#include <opencv2/core.hpp>

#include "../catch2_helpers.h"
#include "../opencv_helpers.h"
#include "../result_helpers.h"
#include "common_error_codes.h"
#include "io/image/Format.h"
#include "io/limits.h"
#include "io/mat.h"
#include "mesh/encode.h"

TEST_CASE("transcode roundtrip", "[mesh][encode]") {
    mesh::Simple mesh;

    mesh.positions.push_back(glm::dvec3(0, 0, 0));
    mesh.positions.push_back(glm::dvec3(1, 0, 0));
    mesh.positions.push_back(glm::dvec3(0, 1, 0));
    mesh.positions.push_back(glm::dvec3(1, 1, 0));

    mesh.triangles.push_back(glm::uvec3(0, 2, 1));
    mesh.triangles.push_back(glm::uvec3(1, 2, 3));

    mesh.uvs.push_back(glm::dvec2(0, 0));
    mesh.uvs.push_back(glm::dvec2(1, 0));
    mesh.uvs.push_back(glm::dvec2(0, 1));
    mesh.uvs.push_back(glm::dvec2(1, 1));

    mesh.texture = cv::Mat3b(100, 100);
    cv::randu(*mesh.texture, cv::Scalar(0, 0, 0), cv::Scalar(256, 256, 256));

    const mesh::Encoded encoded = REQUIRE_VALUE(mesh::encode(mesh, mesh::EncodeOptions{.texture_format = io::image::Format::Png}));

    const mesh::Simple roundtrip_mesh = REQUIRE_VALUE(mesh::decode(encoded, mesh::DecodeOptions{}));
    CHECK(roundtrip_mesh.positions == mesh.positions);
    CHECK(roundtrip_mesh.uvs == mesh.uvs);
    CHECK(roundtrip_mesh.triangles == mesh.triangles);
    REQUIRE(roundtrip_mesh.texture.has_value());
    CHECK(mat_equals(*roundtrip_mesh.texture, *mesh.texture));
}

TEST_CASE("decode reads back an encoded empty mesh", "[mesh][encode]") {
    const mesh::Simple mesh;

    const mesh::Encoded encoded = REQUIRE_VALUE(mesh::encode(mesh));

    const mesh::Simple roundtrip_mesh = REQUIRE_VALUE(mesh::decode(encoded));
    CHECK(roundtrip_mesh.positions == std::vector<glm::dvec3>{});
    CHECK(roundtrip_mesh.uvs == std::vector<glm::dvec2>{});
    CHECK(roundtrip_mesh.triangles == std::vector<glm::uvec3>{});
    CHECK(!roundtrip_mesh.texture.has_value());
}

TEST_CASE("A failed decode originates in decode", "[mesh][encode]") {
    mesh::Encoded encoded;
    encoded.header.version = 2;

    const mesh::DecodeError error = REQUIRE_ERROR(mesh::decode(encoded));

    CHECK(error.holds_code(mesh::GeometryDecodeError::IncompatibleData));
    CHECK(std::string_view(error.origin().file_name()).ends_with("mesh/encode.h"));
}

TEST_CASE("Decoding a mesh whose vertex count exceeds the buffer limit fails, naming the limit and the size", "[mesh][encode]") {
    mesh::Simple mesh;
    mesh.positions = {glm::dvec3(0, 0, 0), glm::dvec3(1, 0, 0), glm::dvec3(0, 1, 0)};
    mesh.triangles = {glm::uvec3(0, 1, 2)};
    mesh::Encoded encoded = REQUIRE_VALUE(mesh::encode(mesh));
    encoded.header.vertex_count = 0xFFFFFFFF;

    const mesh::DecodeError error = REQUIRE_ERROR(mesh::decode(encoded));

    CHECK(error.holds_code(io::BufferLimitExceeded(io::max_buffer_size, size_t{0xFFFFFFFF} * sizeof(glm::dvec3))));
}

TEST_CASE("Decoding a mesh whose texture is not an image fails as corrupt data", "[mesh][encode]") {
    mesh::Simple mesh;
    mesh.positions = {glm::dvec3(0, 0, 0), glm::dvec3(1, 0, 0), glm::dvec3(0, 1, 0)};
    mesh.triangles = {glm::uvec3(0, 1, 2)};
    mesh::Encoded encoded = REQUIRE_VALUE(mesh::encode(mesh));
    encoded.texture = {1, 2, 3, 4};

    const mesh::DecodeError error = REQUIRE_ERROR(mesh::decode(encoded));

    CHECK(error.holds<io::mat::CorruptData>());
}
