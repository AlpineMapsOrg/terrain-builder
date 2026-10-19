#include <catch2/catch_test_macros.hpp>

#include <string_view>

#include <glm/glm.hpp>
#include <opencv2/core.hpp>

#include "../catch2_helpers.h"
#include "../opencv_helpers.h"
#include "../result_helpers.h"
#include "io/UnsupportedFormat.h"
#include "mesh/encode.h"
#include "texture/io.h"

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

    const mesh::Encoded encoded = REQUIRE_VALUE(mesh::encode(mesh, mesh::EncodeOptions{.texture_format = ".png"}));

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

    CHECK(error.holds(mesh::GeometryDecodeError::IncompatibleData));
    CHECK(std::string_view(error.origin().file_name()).ends_with("mesh/encode.h"));
}

TEST_CASE("Encoding a mesh with a texture format that names no image format fails, naming the extension", "[mesh][encode]") {
    mesh::Simple mesh;
    mesh.positions = {glm::dvec3(0, 0, 0), glm::dvec3(1, 0, 0), glm::dvec3(0, 1, 0)};
    mesh.triangles = {glm::uvec3(0, 1, 2)};
    mesh.uvs = {glm::dvec2(0, 0), glm::dvec2(1, 0), glm::dvec2(0, 1)};
    mesh.texture = cv::Mat3b(4, 4, cv::Vec3b(1, 2, 3));

    const mesh::EncodeError error = REQUIRE_ERROR(mesh::encode(mesh, mesh::EncodeOptions{.texture_format = ".not_an_image"}));

    CHECK(error.holds(io::UnsupportedFormat{".not_an_image"}));
}

TEST_CASE("Decoding a mesh whose texture is not an image fails with a texture decode error", "[mesh][encode]") {
    mesh::Simple mesh;
    mesh.positions = {glm::dvec3(0, 0, 0), glm::dvec3(1, 0, 0), glm::dvec3(0, 1, 0)};
    mesh.triangles = {glm::uvec3(0, 1, 2)};
    mesh::Encoded encoded = REQUIRE_VALUE(mesh::encode(mesh));
    encoded.texture = {1, 2, 3, 4};

    const mesh::DecodeError error = REQUIRE_ERROR(mesh::decode(encoded));

    CHECK(error.holds<texture::io::DecodeError>());
}
