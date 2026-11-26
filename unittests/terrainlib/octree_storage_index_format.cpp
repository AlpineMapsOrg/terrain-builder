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
#include <limits>
#include <optional>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "enumerate.h"
#include "io/WriteMode.h"
#include "io/envelope.h"
#include "octree/Id.h"
#include "octree/storage/Index.h"
#include "octree/storage/index_format.h"
#include "store/Hash.h"
#include "store/Index.h"

namespace {

// A copy of the layout of version 1 of the octree index file, so that tests can write files that write_index never would.
namespace v1 {

    struct IndexEntry {
        uint8_t level;
        uint64_t index;
        uint8_t status;
    };

    struct HashEntry {
        uint8_t level;
        uint64_t index;
        store::PayloadHash hash;
    };

    struct StoreIndex {
        std::vector<IndexEntry> entries;
        std::vector<HashEntry> hashes;
    };

} // namespace v1

using StoreIndexSchema = io::envelope::PayloadSchema<"octree.StoreIndex", io::envelope::Version<1, v1::StoreIndex>>;

} // namespace

TEST_CASE("octree indexes round-trip sparse hashes", "[octree][storage][index_format]")
{
    const TempDirectory directory("index-round-trip");
    const octree::Id root = octree::Id::root();
    const std::vector children { root.child(0).value(), root.child(5).value(), root.child(7).value() };
    std::vector<store::PayloadHash> hashes;
    octree::storage::Index index;
    for (const auto [position, child] : enumerate<uint8_t>(children)) {
        hashes.push_back(store::PayloadHash(4 + position, position));
        REQUIRE(index.add(child, hashes.back()));
    }

    REQUIRE_VALUE(octree::storage::write_index(directory.path(), index));
    const octree::storage::Index decoded = REQUIRE_VALUE(octree::storage::read_index(directory.path()));

    for (const auto [position, child] : enumerate(children)) {
        const std::optional<store::PayloadHashView> hash_opt = decoded.hash(child);
        REQUIRE(hash_opt.has_value());
        CHECK(std::ranges::equal(*hash_opt, hashes[position]));
    }
    CHECK_FALSE(decoded.hash(root).has_value());
}

TEST_CASE("octree index decoding rejects invalid ids and duplicate and invalid hash keys", "[octree][storage][index_format]")
{
    const TempDirectory directory("index-decoding");
    const std::filesystem::path index_path = directory.path() / octree::storage::index_file_name;
    const octree::Id child = octree::Id::root().child(2).value();
    octree::storage::Index index;
    REQUIRE(index.add(child, store::PayloadHash(8, 1)));
    REQUIRE_VALUE(octree::storage::write_index(directory.path(), index));
    v1::StoreIndex encoded = REQUIRE_VALUE(io::envelope::read<StoreIndexSchema>(index_path));
    REQUIRE(encoded.hashes.size() == 1);

    SECTION("invalid id") { encoded.entries.front().level = std::numeric_limits<uint8_t>::max(); }
    SECTION("duplicate hash key") { encoded.hashes.push_back(encoded.hashes.front()); }
    SECTION("invalid hash key") { encoded.hashes.front().level = std::numeric_limits<uint8_t>::max(); }
    REQUIRE_VALUE(io::envelope::write<StoreIndexSchema>(encoded, index_path, { .mode = io::WriteMode::Overwrite }));

    CHECK(REQUIRE_ERROR(octree::storage::read_index(directory.path())).holds<store::CorruptIndex>());
}
