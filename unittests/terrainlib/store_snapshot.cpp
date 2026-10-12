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

#include <filesystem>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "../temporary_directory.h"
#include "io/byte.h"
#include "io/envelope.h"
#include "mesh/storage.h"
#include "octree/storage/snapshot.h"
#include "octree/store_layout/Mappings.h"
#include "store/inputs.h"
#include "store/snapshot.h"

namespace {

using test::TemporaryDirectory;

struct Record {
    std::string input;
    int setting = 0;

    bool operator==(const Record&) const = default;
};

using Schema = io::envelope::PayloadSchema<"test.Inputs", io::envelope::Version<1, Record>>;

mesh::Simple triangle() { return mesh::Simple({ { 0, 1, 2 } }, { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 } }); }

Expected<mesh::storage::IndexedStorage> create_mesh_storage(const std::filesystem::path& final_path)
{
    return octree::storage::create<mesh::Simple>(
        final_path, std::string(mesh::storage::payload_class), ".sfmesh", mesh::codec::from_extension, octree::store_layout::flat());
}

} // namespace

TEST_CASE("snapshots are created as .part and published by renaming", "[store][snapshot]")
{
    TemporaryDirectory directory("snapshot");
    const auto final_path = directory.path() / "nested" / "output";
    const auto partial_path = store::snapshot::partial_path(final_path);
    CHECK(partial_path.filename() == "output.part");
    CHECK(store::snapshot::is_partial(partial_path));
    CHECK_FALSE(store::snapshot::is_partial(final_path));

    const auto created = store::snapshot::create_partial(final_path);
    REQUIRE(created.has_value());
    CHECK(*created == partial_path);
    CHECK(std::filesystem::is_directory(partial_path));

    SECTION("an existing .part is rejected")
    {
        const auto again = store::snapshot::create_partial(final_path);
        REQUIRE_FALSE(again.has_value());
        CHECK(again.error().code() == Error::Code::AlreadyExists);
    }

    SECTION("a .part final path is rejected")
    {
        const auto rejected = store::snapshot::create_partial(partial_path);
        REQUIRE_FALSE(rejected.has_value());
        CHECK(rejected.error().code() == Error::Code::InvalidInput);
    }

    SECTION("publication removes the input record and renames")
    {
        REQUIRE(store::inputs::write<Schema>(partial_path, Record { "input", 1 }).has_value());
        REQUIRE(std::filesystem::exists(partial_path / store::inputs::file_name));
        REQUIRE(store::snapshot::publish(partial_path).has_value());
        CHECK_FALSE(std::filesystem::exists(partial_path));
        CHECK(std::filesystem::is_directory(final_path));
        CHECK_FALSE(std::filesystem::exists(final_path / store::inputs::file_name));

        const auto again = store::snapshot::create_partial(final_path);
        REQUIRE_FALSE(again.has_value());
        CHECK(again.error().code() == Error::Code::AlreadyExists);
    }

    SECTION("publication does not replace an existing destination")
    {
        SECTION("empty directory") { REQUIRE(std::filesystem::create_directory(final_path)); }
        SECTION("nonempty directory") { REQUIRE(io::byte::write(io::byte::span {}, final_path / "keep")); }
        SECTION("dangling symlink") { std::filesystem::create_symlink(directory.path() / "missing", final_path); }
        const auto published = store::snapshot::publish(partial_path);
        REQUIRE_FALSE(published.has_value());
        CHECK(published.error().code() == Error::Code::AlreadyExists);
        CHECK(std::filesystem::is_directory(partial_path));
    }
}

TEST_CASE("caches must be .part snapshots with a matching input record", "[store][snapshot]")
{
    TemporaryDirectory directory("snapshot-cache");
    const Record record { "input", 1 };
    const auto cache = directory.path() / "cache.part";
    REQUIRE(std::filesystem::create_directory(cache));

    SECTION("missing record")
    {
        const auto validated = store::inputs::validate_cache<Schema>(cache, record);
        REQUIRE_FALSE(validated.has_value());
    }

    SECTION("matching record")
    {
        REQUIRE(store::inputs::write<Schema>(cache, record).has_value());
        CHECK(store::inputs::validate_cache<Schema>(cache, record).has_value());
        CHECK(store::inputs::validate_cache<Schema>(cache.string() + "/", record).has_value());
    }

    SECTION("different record")
    {
        REQUIRE(store::inputs::write<Schema>(cache, record).has_value());
        const auto validated = store::inputs::validate_cache<Schema>(cache, Record { "input", 2 });
        REQUIRE_FALSE(validated.has_value());
        CHECK(validated.error().code() == Error::Code::InvalidInput);
    }

    SECTION("not a .part snapshot")
    {
        const auto published = directory.path() / "published";
        REQUIRE(std::filesystem::create_directory(published));
        REQUIRE(store::inputs::write<Schema>(published, record).has_value());
        const auto validated = store::inputs::validate_cache<Schema>(published, record);
        REQUIRE_FALSE(validated.has_value());
        CHECK(validated.error().code() == Error::Code::InvalidInput);
    }
}

TEST_CASE("octree snapshots are created empty, published, and fingerprinted", "[store][snapshot][octree]")
{
    TemporaryDirectory directory("octree-snapshot");
    const auto final_path = directory.path() / "output";
    const octree::Id root = octree::Id::root();
    {
        auto created = create_mesh_storage(final_path);
        REQUIRE(created.has_value());
        auto storage = std::move(*created);
        CHECK(storage.base_path() == store::snapshot::partial_path(final_path));
        CHECK(std::filesystem::is_regular_file(storage.base_path() / octree::storage::index_file_name));
        REQUIRE(storage.save(root, triangle()).has_value());
        REQUIRE(octree::storage::publish(std::move(storage)).has_value());
    }
    CHECK_FALSE(std::filesystem::exists(store::snapshot::partial_path(final_path)));

    const auto reopened = mesh::storage::open_index(final_path / octree::storage::index_file_name);
    REQUIRE(reopened.has_value());
    CHECK(reopened->has(root).value());
    CHECK(reopened->codec_selector() == ".sfmesh");

    const auto existing = create_mesh_storage(final_path);
    REQUIRE_FALSE(existing.has_value());
    CHECK(existing.error().code() == Error::Code::AlreadyExists);

    const auto fingerprint = store::inputs::fingerprint(final_path, octree::storage::metadata_file_name, octree::storage::index_file_name);
    REQUIRE(fingerprint.has_value());
    CHECK(fingerprint->path == std::filesystem::canonical(final_path).string());
    CHECK(fingerprint->index_hash == io::envelope::read_header(final_path / octree::storage::index_file_name)->hash);
    CHECK(fingerprint->metadata_hash == io::envelope::read_header(final_path / octree::storage::metadata_file_name)->hash);

    const auto metadata_path = final_path / octree::storage::metadata_file_name;
    const auto metadata = io::envelope::read_from_path<octree::storage::StoreMetadataSchema>(metadata_path);
    REQUIRE(metadata.has_value());
    SECTION("recompression keeps the fingerprint")
    {
        REQUIRE(io::envelope::write_to_path<octree::storage::StoreMetadataSchema>(*metadata, metadata_path, true, io::envelope::CompressionAlgorithm::None));
        const auto recompressed = store::inputs::fingerprint(final_path, octree::storage::metadata_file_name, octree::storage::index_file_name);
        REQUIRE(recompressed.has_value());
        CHECK(*recompressed == *fingerprint);
    }
    SECTION("files without a hash are rejected")
    {
        REQUIRE(io::envelope::write_to_path<octree::storage::StoreMetadataSchema>(
            *metadata, metadata_path, true, io::envelope::CompressionAlgorithm::ZstdDefaultCompression, io::hash::Algorithm::None));
        const auto unhashed = store::inputs::fingerprint(final_path, octree::storage::metadata_file_name, octree::storage::index_file_name);
        REQUIRE_FALSE(unhashed.has_value());
        CHECK(unhashed.error().code() == Error::Code::Unsupported);
    }
}
