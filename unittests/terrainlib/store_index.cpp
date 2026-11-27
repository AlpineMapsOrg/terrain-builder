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

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "hierarchy/IndexMap.h"
#include "octree/Id.h"
#include "octree/StoreTraits.h"
#include "octree/storage/Index.h"
#include "store/Hash.h"
#include "store/Index.h"
#include "store_test_utils.h"

TEST_CASE("validate_index requires every hash, of at least min_hash_size bytes, exactly when the codec hashes", "[store][Index]")
{
    const octree::Id child = octree::Id::root().child(1).value();
    hierarchy::IndexMap<octree::KeyTraits, octree::HierarchyTraits> map;
    REQUIRE(map.add(child));
    octree::storage::Index::HashMap hashes { { child, store_test::int_hash(1) } };
    REQUIRE_VALUE(store::validate_index(REQUIRE_VALUE(octree::storage::Index::from_parts(map, hashes)), true));
    REQUIRE_VALUE(store::validate_index(REQUIRE_VALUE(octree::storage::Index::from_parts(map, {})), false));

    bool codec_supports_hashing = true;
    SECTION("missing hash") { hashes.clear(); }
    SECTION("short hash") { hashes[child].resize(store::min_hash_size - 1); }
    SECTION("hash with a codec that does not support hashing") { codec_supports_hashing = false; }
    const octree::storage::Index index = REQUIRE_VALUE(octree::storage::Index::from_parts(map, hashes));
    CHECK(REQUIRE_ERROR(store::validate_index(index, codec_supports_hashing)).holds<store::CorruptIndex>());
}
