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

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <radix/tile.h>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "hierarchy/NodeStatus.h"
#include "io/WriteMode.h"
#include "io/envelope.h"
#include "raster_store/StoreTraits.h"
#include "raster_store/storage/Index.h"
#include "raster_store/storage/index_format.h"
#include "store/Hash.h"
#include "store/Index.h"

namespace {

namespace storage = raster_store::storage;

// A copy of the layout of version 1 of the raster index file, so that tests can write files that write_index never would.
namespace v1 {

    struct IndexEntry {
        radix::tile::Id id;
        uint8_t status;
    };

    struct HashEntry {
        radix::tile::Id id;
        store::PayloadHash hash;
    };

    struct Hierarchy {
        std::vector<IndexEntry> entries;
        std::vector<HashEntry> hashes;
    };

} // namespace v1

using IndexSchema = io::envelope::PayloadSchema<"raster_store.Index", io::envelope::Version<1, v1::Hierarchy>>;

} // namespace

TEST_CASE("raster index adapters preserve mixed hierarchy and reject corrupt topology", "[raster_store][storage][index_format]")
{
    const TempDirectory directory("raster-index-topology");
    const std::filesystem::path index_path = directory.path() / storage::index_file_name;
    const radix::tile::Id root = raster_store::HierarchyTraits::root();
    storage::Index index;
    REQUIRE(index.add(root, std::nullopt));
    REQUIRE(index.add({ 2, { 1, 1 } }, std::nullopt));
    REQUIRE_VALUE(storage::write_index(directory.path(), index));

    const storage::Index decoded = REQUIRE_VALUE(storage::read_index(directory.path()));
    CHECK(decoded.status(root) == hierarchy::NodeStatus::Inner);
    CHECK(decoded.status({ 1, { 0, 0 } }) == hierarchy::NodeStatus::Virtual);
    CHECK(decoded.status({ 2, { 1, 1 } }) == hierarchy::NodeStatus::Leaf);
    v1::Hierarchy encoded = REQUIRE_VALUE(io::envelope::read<IndexSchema>(index_path));
    REQUIRE(encoded.entries.size() == 3);
    // The sections below pick entries by position.
    REQUIRE(encoded.entries[0].id == root);
    REQUIRE(encoded.entries[1].id == radix::tile::Id { 1, { 0, 0 } });
    REQUIRE(encoded.entries[2].id == radix::tile::Id { 2, { 1, 1 } });

    SECTION("duplicate key") { encoded.entries.push_back(encoded.entries[0]); }
    SECTION("invalid key") { encoded.entries[0].id.coords.x = 1; }
    SECTION("invalid status") { encoded.entries[0].status = 255; }
    SECTION("missing parent") { encoded.entries.erase(encoded.entries.begin() + 1); }
    SECTION("leaf with descendant") { encoded.entries[0].status = hierarchy::NodeStatus::Leaf; }
    SECTION("virtual without children") { encoded.entries.pop_back(); }
    REQUIRE_VALUE(io::envelope::write<IndexSchema>(encoded, index_path, { .mode = io::WriteMode::Overwrite }));
    CHECK(REQUIRE_ERROR(storage::read_index(directory.path())).holds<store::CorruptIndex>());
}

TEST_CASE("raster index adapters round-trip tile hashes and reject duplicate and invalid keys", "[raster_store][storage][index_format]")
{
    const TempDirectory directory("raster-index-hashes");
    const std::filesystem::path index_path = directory.path() / storage::index_file_name;
    const radix::tile::Id first { 1, { 1, 0 } };
    const radix::tile::Id second { 1, { 0, 1 } };
    const store::PayloadHash first_hash(5, 1);
    const store::PayloadHash second_hash(8, 2);
    storage::Index index;
    REQUIRE(index.add(second, second_hash));
    REQUIRE(index.add(first, first_hash));
    REQUIRE_VALUE(storage::write_index(directory.path(), index));

    v1::Hierarchy encoded = REQUIRE_VALUE(io::envelope::read<IndexSchema>(index_path));
    REQUIRE(encoded.hashes.size() == 2);
    const storage::Index decoded = REQUIRE_VALUE(storage::read_index(directory.path()));
    const std::optional<store::PayloadHashView> first_hash_opt = decoded.hash(first);
    const std::optional<store::PayloadHashView> second_hash_opt = decoded.hash(second);
    REQUIRE(first_hash_opt.has_value());
    REQUIRE(second_hash_opt.has_value());
    CHECK(std::ranges::equal(*first_hash_opt, first_hash));
    CHECK(std::ranges::equal(*second_hash_opt, second_hash));

    SECTION("duplicate key") { encoded.hashes.push_back(encoded.hashes.front()); }
    SECTION("invalid key") { encoded.hashes.front().id.coords.x = 2; }
    REQUIRE_VALUE(io::envelope::write<IndexSchema>(encoded, index_path, { .mode = io::WriteMode::Overwrite }));
    CHECK(REQUIRE_ERROR(storage::read_index(directory.path())).holds<store::CorruptIndex>());
}
