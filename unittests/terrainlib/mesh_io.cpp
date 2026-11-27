/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2022 Adam Celarek-Litofcenko
 * Copyright (C) 2022 Martin Braunsperger
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
#include <fstream>
#include <numbers>
#include <string>
#include <string_view>
#include <system_error>

#include <opencv2/opencv.hpp>
#include <fmt/format.h>

#include "../catch2_helpers.h"
#include "../opencv_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "common_error_codes.h"
#include "error/format.h"
#include "io/FileError.h"
#include "io/envelope.h"
#include "io/filesystem.h"
#include "io/image/Format.h"
#include "mesh/io.h"
#include "mesh/io/gltf.h"

TEST_CASE("Saving and loading a mesh round-trips it", "[mesh][io]") {
    for (const auto &format : {"gltf", "glb", "sfmesh"}) {
        DYNAMIC_SECTION(format) {
            mesh::Simple mesh;

            mesh.positions.push_back(glm::dvec3(0, 0, 0));
            mesh.positions.push_back(glm::dvec3(1, 0, 0));
            mesh.positions.push_back(glm::dvec3(0, 1, 0));
            mesh.positions.push_back(glm::dvec3(1, 1, 0));

            mesh.triangles.push_back(glm::uvec3(0, 2, 1));
            mesh.triangles.push_back(glm::uvec3(1, 2, 3));

            SECTION("textured") {
                mesh.uvs.push_back(glm::dvec2(0, 0));
                mesh.uvs.push_back(glm::dvec2(1, 0));
                mesh.uvs.push_back(glm::dvec2(0, 1));
                mesh.uvs.push_back(glm::dvec2(1, 1));

                mesh.texture = cv::Mat3b(100, 100);
                cv::randu(*mesh.texture, cv::Scalar(0, 0, 0), cv::Scalar(256, 256, 256));
            }

            SECTION("no texture") {
                mesh.uvs.push_back(glm::dvec2(0, 0));
                mesh.uvs.push_back(glm::dvec2(1, 0));
                mesh.uvs.push_back(glm::dvec2(0, 1));
                mesh.uvs.push_back(glm::dvec2(1, 1));
            }

            SECTION("no texture and no uvs") {
            }

            if (std::string_view(format) == "sfmesh") {
                SECTION("positions beyond float precision") {
                    const double pi = std::numbers::pi_v<double>;
                    CHECK(static_cast<double>(static_cast<float>(pi)) != pi);
                    mesh.positions[1] = glm::dvec3(pi, 0, 0);
                    mesh.positions[2] = glm::dvec3(0, pi, 0);
                }
            }

            const TempDirectory directory("mesh_io_roundtrip");
            const std::filesystem::path mesh_path = directory.path() / fmt::format("mesh.{}", format);

            REQUIRE_VALUE(mesh::io::save_to_path(mesh, mesh_path, mesh::io::SaveOptions{.texture_format = io::image::Format::Png}));

            const mesh::Simple roundtrip_mesh = REQUIRE_VALUE(mesh::io::load_from_path(mesh_path));
            CHECK(roundtrip_mesh.positions == mesh.positions);
            CHECK(roundtrip_mesh.uvs == mesh.uvs);
            CHECK(roundtrip_mesh.triangles == mesh.triangles);
            REQUIRE(roundtrip_mesh.texture.has_value() == mesh.texture.has_value());
            if (mesh.texture.has_value()) {
                CHECK(mat_equals(*roundtrip_mesh.texture, *mesh.texture));
            }
        }
    }
}

namespace {

// A mesh of a single triangle.
mesh::Simple make_triangle() {
    mesh::Simple mesh;
    mesh.positions.push_back(glm::dvec3(0, 0, 0));
    mesh.positions.push_back(glm::dvec3(1, 0, 0));
    mesh.positions.push_back(glm::dvec3(0, 1, 0));
    mesh.triangles.push_back(glm::uvec3(0, 1, 2));
    return mesh;
}

} // namespace

TEST_CASE("Loading a mesh with an unknown extension fails as unsupported", "[mesh][io]") {
    const auto error = REQUIRE_ERROR(mesh::io::load_from_path("mesh.xyz"));

    CHECK(error.holds<Unsupported>());
}

TEST_CASE("Saving a mesh with an unknown extension fails as unsupported", "[mesh][io]") {
    const TempDirectory directory("mesh_io_unsupported_save");
    const auto error = REQUIRE_ERROR(mesh::io::save_to_path(make_triangle(), directory.path() / "mesh.xyz"));

    CHECK(error.holds<Unsupported>());
}

TEST_CASE("Loading a missing mesh file reports that it does not exist", "[mesh][io]") {
    const TempDirectory directory("mesh_io_missing");
    const std::string extension = GENERATE(".sfmesh", ".gltf", ".glb");
    const std::filesystem::path path = directory.path() / fmt::format("missing{}", extension);

    const auto error = REQUIRE_ERROR(mesh::io::load_from_path(path));
    CHECK(error.holds_code(std::errc::no_such_file_or_directory));
}

TEST_CASE("Loading a truncated SF mesh file fails as corrupt data", "[mesh][io]") {
    const TempDirectory directory("mesh_io_truncated_sf_mesh");
    const std::filesystem::path path = directory.path() / "mesh.sfmesh";
    REQUIRE_VALUE(mesh::io::save_to_path(make_triangle(), path));
    const uint64_t size = REQUIRE_VALUE(io::filesystem::file_size(path));
    REQUIRE_VALUE(io::filesystem::resize_file(path, size - 1));

    const auto error = REQUIRE_ERROR(mesh::io::load_from_path(path));

    CHECK(error.holds<io::envelope::CorruptData>());
}

TEST_CASE("Loading a glTF file without a mesh fails as unsupported", "[mesh][io]") {
    const TempDirectory directory("mesh_io_incomplete_gltf");
    const std::filesystem::path path = directory.path() / "empty.gltf";
    std::ofstream(path) << R"({"asset":{"version":"2.0"}})";

    const auto error = REQUIRE_ERROR(mesh::io::load_from_path(path));

    CHECK(error.holds<Unsupported>());
}

TEST_CASE("Loading a file that is not glTF fails as corrupt data", "[mesh][io]") {
    const TempDirectory directory("mesh_io_corrupt_gltf");
    const std::filesystem::path path = directory.path() / "corrupt.gltf";
    std::ofstream(path) << "not glTF";

    const auto error = REQUIRE_ERROR(mesh::io::load_from_path(path));

    CHECK(error.holds<mesh::io::gltf::CorruptData>());
}

TEST_CASE("Saving a mesh onto a directory fails with a file error", "[mesh][io]") {
    const TempDirectory directory("mesh_io_directory_save");
    const std::filesystem::path path = directory.path() / "directory.glb";
    REQUIRE(REQUIRE_VALUE(io::filesystem::create_directory(path)));

    const auto error = REQUIRE_ERROR(mesh::io::save_to_path(make_triangle(), path));

    CHECK(error.holds<io::FileError>());
}
