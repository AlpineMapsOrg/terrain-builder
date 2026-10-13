/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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

#include <atomic>
#include <filesystem>
#include <future>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "../temporary_directory.h"
#include "io/byte.h"
#include "io/envelope.h"
#include "mesh/codec/Gltf.h"
#include "mesh/codec/SfMesh.h"
#include "store/Codec.h"

namespace {

using test::TemporaryDirectory;

class MultiFileCodec final : public store::Codec<int> {
public:
    std::vector<std::filesystem::path> paths(const std::filesystem::path& node_path) const override
    {
        return { add_extension(node_path, ".data"), add_extension(node_path, ".metadata") };
    }
};

class WriteOnlyCodec final : public store::Codec<int> {
public:
    std::vector<std::filesystem::path> paths(const std::filesystem::path& node_path) const override { return { add_extension(node_path, ".out") }; }
    Expected<store::NodeDataHash> write(const std::filesystem::path&, const int&) const override
    {
        ++writes;
        return store::NodeDataHash {};
    }

    mutable std::atomic_uint32_t writes = 0;
};

mesh::Simple triangle_mesh(const double offset)
{
    return mesh::Simple({ { 0, 1, 2 } }, { { offset, 0.0, 0.0 }, { offset + 1.0, 0.0, 0.0 }, { offset, 1.0, 0.0 } });
}

template <typename Codec>
void check_mesh_codec_reentrancy(const Codec& codec, const std::filesystem::path& base)
{
    std::vector<std::future<bool>> writes;
    for (int index = 0; index < 4; ++index) {
        writes.push_back(std::async(
            std::launch::async, [&codec, base, index] { return codec.write(base / std::to_string(index) / "node", triangle_mesh(index)).has_value(); }));
    }
    for (auto& write : writes) {
        REQUIRE(write.get());
    }

    std::vector<std::future<bool>> reads;
    for (int index = 0; index < 4; ++index) {
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
    REQUIRE(codec.write(node_path, triangle_mesh(0.0)).has_value());
    const auto path = codec.paths(node_path).front();
    const auto original = io::byte::read(path);
    REQUIRE(original.has_value());

    const auto rejected = codec.write(node_path, triangle_mesh(5.0));
    REQUIRE_FALSE(rejected.has_value());
    CHECK(rejected.error().code() == Error::Code::AlreadyExists);
    CHECK(io::byte::read(path).value() == original.value());
}

} // namespace

TEST_CASE("runtime codec defaults report unsupported operations", "[store][codec]")
{
    MultiFileCodec codec;
    const std::filesystem::path path("12/34/56/78");
    CHECK(codec.paths(path)
        == std::vector<std::filesystem::path> {
            "12/34/56/78.data",
            "12/34/56/78.metadata",
        });

    const auto read = codec.read(path);
    REQUIRE_FALSE(read.has_value());
    CHECK(read.error().code() == Error::Code::Unsupported);

    const auto write = codec.write(path, 42);
    REQUIRE_FALSE(write.has_value());
    CHECK(write.error().code() == Error::Code::Unsupported);
}

TEST_CASE("runtime write-only codec remains explicitly unreadable", "[store][codec]")
{
    WriteOnlyCodec codec;
    REQUIRE(codec.write("node", 42).has_value());
    CHECK(codec.writes == 1);
    const auto read = codec.read("node");
    REQUIRE_FALSE(read.has_value());
    CHECK(read.error().code() == Error::Code::Unsupported);
}

TEST_CASE("runtime mesh codecs are reentrant", "[store][codec]")
{
    TemporaryDirectory directory("mesh-reentrant");

    SECTION("SF mesh") { check_mesh_codec_reentrancy(mesh::codec::SfMesh {}, directory.path() / "sfmesh"); }
    SECTION("binary glTF") { check_mesh_codec_reentrancy(mesh::codec::Gltf(mesh::codec::GltfContainer::Binary), directory.path() / "glb"); }
    SECTION("JSON glTF") { check_mesh_codec_reentrancy(mesh::codec::Gltf(mesh::codec::GltfContainer::Json), directory.path() / "gltf"); }
}

TEST_CASE("runtime mesh codecs never replace existing files", "[store][codec]")
{
    TemporaryDirectory directory("mesh-exclusive");

    SECTION("SF mesh") { check_mesh_codec_creates_exclusively(mesh::codec::SfMesh {}, directory.path() / "node"); }
    SECTION("binary glTF") { check_mesh_codec_creates_exclusively(mesh::codec::Gltf(mesh::codec::GltfContainer::Binary), directory.path() / "node"); }
    SECTION("JSON glTF") { check_mesh_codec_creates_exclusively(mesh::codec::Gltf(mesh::codec::GltfContainer::Json), directory.path() / "node"); }
}

TEST_CASE("runtime glTF codec reports writer IO errors", "[store][codec]")
{
    TemporaryDirectory directory("gltf-error");
    mesh::codec::Gltf codec(mesh::codec::GltfContainer::Binary);
    const std::filesystem::path blocker(directory.path() / "blocker");
    REQUIRE(io::byte::write(io::byte::span {}, blocker, io::byte::WriteMode::Overwrite).has_value());

    const auto result = codec.write(blocker / "node", triangle_mesh(0.0));
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code() == Error::Code::Io);
}

TEST_CASE("SF mesh codec returns and verifies its payload hash", "[store][codec][hash]")
{
    TemporaryDirectory directory("sfmesh-hash");
    const mesh::codec::SfMesh codec;
    CHECK(codec.supports_hashing());
    const std::filesystem::path node_path = directory.path() / "node";
    const auto hash = codec.write(node_path, triangle_mesh(0.0));
    REQUIRE(hash.has_value());
    CHECK(*hash == io::envelope::read_header(codec.paths(node_path).front())->hash);
    CHECK(codec.read(node_path, *hash).has_value());

    REQUIRE(std::filesystem::remove(codec.paths(node_path).front()));
    const auto other_hash = codec.write(node_path, triangle_mesh(1.0));
    REQUIRE(other_hash.has_value());
    CHECK(*other_hash != *hash);
    const auto mismatched = codec.read(node_path, *hash);
    REQUIRE_FALSE(mismatched.has_value());
    CHECK(mismatched.error().code() == Error::Code::CorruptData);
}
