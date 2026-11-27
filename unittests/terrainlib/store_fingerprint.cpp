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

#include <optional>

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "octree/Id.h"
#include "octree/storage/Index.h"
#include "store/fingerprint.h"
#include "store_test_utils.h"

TEST_CASE("fingerprint is equal for equal content and differs when a key or a hash differs", "[store][fingerprint]")
{
    const octree::Id first = octree::Id::root().child(1).value();
    const octree::Id second = octree::Id::root().child(2).value();
    const octree::Id third = octree::Id::root().child(3).value();
    octree::storage::Index index;
    REQUIRE(index.add(first, store_test::int_hash(1)));
    REQUIRE(index.add(second, store_test::int_hash(2)));
    const store::Fingerprint fingerprint = REQUIRE_VALUE(store::fingerprint(index));

    octree::storage::Index other;
    SECTION("equal content added in another order")
    {
        REQUIRE(other.add(second, store_test::int_hash(2)));
        REQUIRE(other.add(first, store_test::int_hash(1)));
        CHECK(REQUIRE_VALUE(store::fingerprint(other)) == fingerprint);
    }

    SECTION("a different hash")
    {
        REQUIRE(other.add(first, store_test::int_hash(1)));
        REQUIRE(other.add(second, store_test::int_hash(3)));
        CHECK(REQUIRE_VALUE(store::fingerprint(other)) != fingerprint);
    }

    SECTION("a different key")
    {
        REQUIRE(other.add(first, store_test::int_hash(1)));
        REQUIRE(other.add(third, store_test::int_hash(2)));
        CHECK(REQUIRE_VALUE(store::fingerprint(other)) != fingerprint);
    }
}

TEST_CASE("fingerprint fails for a node without a hash", "[store][fingerprint]")
{
    octree::storage::Index index;
    REQUIRE(index.add(octree::Id::root(), std::nullopt));

    CHECK(REQUIRE_ERROR(store::fingerprint(index)).holds_code(store::FingerprintError::MissingHash));
}

TEST_CASE("fingerprint succeeds for an empty index", "[store][fingerprint]")
{
    const octree::storage::Index index;

    REQUIRE_VALUE(store::fingerprint(index));
}
