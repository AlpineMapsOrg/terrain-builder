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

#include <filesystem>
#include <future>
#include <memory>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "common_error_codes.h"
#include "io/FileError.h"
#include "io/bytes.h"
#include "io/envelope.h"
#include "io/filesystem.h"
#include "mesh/SimpleMesh.h"
#include "mesh/codec/Gltf.h"
#include "mesh/codec/SfMesh.h"
#include "mesh/codec/resolve.h"
#include "store/Codec.h"
#include "store/Hash.h"

namespace {

mesh::Simple triangle_mesh(const double offset)
{
    return mesh::Simple({ { 0, 1, 2 } }, { { offset, 0.0, 0.0 }, { offset + 1.0, 0.0, 0.0 }, { offset, 1.0, 0.0 } });
}

template <typename Codec>
void check_mesh_codec_reentrancy(const Codec& codec, const std::filesystem::path& base)
{
    std::vector<std::future<bool>> writes;
    for (int index = 0; index < 4; index++) {
        writes.push_back(std::async(
            std::launch::async, [&codec, base, index] { return codec.write(base / std::to_string(index) / "node", triangle_mesh(index)).has_value(); }));
    }
    for (auto& write : writes) {
        REQUIRE(write.get());
    }

    std::vector<std::future<bool>> reads;
    for (int index = 0; index < 4; index++) {
        reads.push_back(std::async(std::launch::async, [&codec, base, index] {
            const auto result = codec.read(base / std::to_string(index) / "node");
            return result.has_value() && result->face_count() == 1;
        }));
    }
    for (auto& read : reads) {
        REQUIRE(read.get());
    }
}

template <typename Codec>
void check_mesh_codec_creates_exclusively(const Codec& codec, const std::filesystem::path& node_path)
{
    REQUIRE_VALUE(codec.write(node_path, triangle_mesh(0.0)));
    const std::filesystem::path path = codec.paths(node_path).front();
    const io::bytes::vector original = REQUIRE_VALUE(io::bytes::read(path));

    const auto error = REQUIRE_ERROR(codec.write(node_path, triangle_mesh(5.0)));
    CHECK(error.template holds_and<io::FileError>([](const io::FileError& e) { return e.code() == std::errc::file_exists; }));
    CHECK(REQUIRE_VALUE(io::bytes::read(path)) == original);
}

} // namespace

TEST_CASE("runtime mesh codecs are reentrant", "[mesh][codec][SfMesh][Gltf]")
{
    const TempDirectory directory("mesh-reentrant");

    SECTION("SF mesh") { check_mesh_codec_reentrancy(mesh::codec::SfMesh {}, directory.path() / "sfmesh"); }
    SECTION("binary glTF") { check_mesh_codec_reentrancy(mesh::codec::Gltf(mesh::io::gltf::Container::Binary), directory.path() / "glb"); }
    SECTION("JSON glTF") { check_mesh_codec_reentrancy(mesh::codec::Gltf(mesh::io::gltf::Container::Json), directory.path() / "gltf"); }
}

TEST_CASE("runtime mesh codecs never replace existing files", "[mesh][codec][SfMesh][Gltf]")
{
    const TempDirectory directory("mesh-exclusive");

    SECTION("SF mesh") { check_mesh_codec_creates_exclusively(mesh::codec::SfMesh {}, directory.path() / "node"); }
    SECTION("binary glTF") { check_mesh_codec_creates_exclusively(mesh::codec::Gltf(mesh::io::gltf::Container::Binary), directory.path() / "node"); }
    SECTION("JSON glTF") { check_mesh_codec_creates_exclusively(mesh::codec::Gltf(mesh::io::gltf::Container::Json), directory.path() / "node"); }
}

TEST_CASE("runtime mesh codecs own their filename endings", "[mesh][codec][SfMesh][Gltf]")
{
    const std::filesystem::path path("12/34/56/78");
    const mesh::codec::SfMesh sfmesh;
    const mesh::codec::Gltf binary(mesh::io::gltf::Container::Binary);
    const mesh::codec::Gltf json(mesh::io::gltf::Container::Json);

    CHECK(sfmesh.paths(path) == std::vector<std::filesystem::path> { "12/34/56/78.sfmesh" });
    CHECK(binary.paths(path) == std::vector<std::filesystem::path> { "12/34/56/78.glb" });
    CHECK(json.paths(path) == std::vector<std::filesystem::path> { "12/34/56/78.gltf" });
}

TEST_CASE("runtime glTF codec reports writer IO errors", "[mesh][codec][Gltf]")
{
    const TempDirectory directory("gltf-error");
    const mesh::codec::Gltf codec(mesh::io::gltf::Container::Binary);
    const std::filesystem::path blocker = directory.path() / "blocker";
    REQUIRE_VALUE(io::bytes::write(io::bytes::vector {}, blocker));

    const auto error = REQUIRE_ERROR(codec.write(blocker / "node", triangle_mesh(0.0)));
    CHECK(error.holds<io::FileError>());
}

TEST_CASE("SF mesh codec returns and verifies its payload hash", "[mesh][codec][SfMesh]")
{
    const TempDirectory directory("sfmesh-hash");
    const mesh::codec::SfMesh codec;
    CHECK(codec.supports_hashing());
    const std::filesystem::path node_path = directory.path() / "node";
    const store::PayloadHash hash = REQUIRE_VALUE(codec.write(node_path, triangle_mesh(0.0)));
    CHECK(hash == REQUIRE_VALUE(io::envelope::read_header(codec.paths(node_path).front())).hash);
    REQUIRE_VALUE(codec.read(node_path, hash));

    REQUIRE(REQUIRE_VALUE(io::filesystem::remove(codec.paths(node_path).front())));
    const store::PayloadHash other_hash = REQUIRE_VALUE(codec.write(node_path, triangle_mesh(1.0)));
    CHECK(other_hash != hash);
    CHECK(REQUIRE_ERROR(codec.read(node_path, hash)).holds<store::CorruptData>());
}

TEST_CASE("mesh codec resolver dispatches every supported codec id", "[mesh][codec][resolve]")
{
    for (const std::string_view id : { ".sfmesh", ".glb", ".gltf" }) {
        const std::unique_ptr<store::Codec<mesh::Simple>> codec = REQUIRE_VALUE(mesh::codec::resolve(id));
        const std::vector<std::filesystem::path> paths = codec->paths("node");
        REQUIRE_FALSE(paths.empty());
        CHECK(paths.front().extension() == id);
    }

    CHECK(REQUIRE_ERROR(mesh::codec::resolve(".unknown")).holds<Unsupported>());
}
