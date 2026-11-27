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

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string_view>
#include <utility>

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "common_error_codes.h"
#include "octree/Id.h"
#include "octree/StoreTraits.h"
#include "octree/path_layout.h"
#include "store/IndexFormat.h"
#include "store/PersistedIndex.h"
#include "store/SimpleIndex.h"
#include "store/path_layout.h"
#include "store_test_utils.h"

namespace {

using Index = store::SimpleIndex<octree::KeyTraits>;

std::atomic_uint32_t index_writes = 0;

alp::Result<Index, store::ReadIndexError> read_unused_index(const std::filesystem::path&)
{
    return alp::make_error(Unsupported {}, "the test format cannot read an index");
}

alp::Result<void, store::WriteIndexError> count_index_write(const std::filesystem::path&, const Index&)
{
    index_writes++;
    return {};
}

std::optional<store::path_layout::Layout<octree::Id>> resolve_no_layout(std::string_view)
{
    return std::nullopt;
}

store::IndexFormat<Index> counting_index_format()
{
    return {
        read_unused_index,
        count_index_write,
        resolve_no_layout,
        octree::path_layout::flat::layout,
    };
}

} // namespace

TEST_CASE("PersistedIndex moves transfer and finalize dirty index state", "[store][PersistedIndex]")
{
    index_writes = 0;
    const octree::Id root = octree::Id::root();

    {
        store::PersistedIndex<Index> original(Index {}, counting_index_format(), "first", false);
        REQUIRE(original.add(root, store_test::int_hash(1)));
        store::PersistedIndex<Index> moved = std::move(original);
        CHECK(index_writes == 0);
        REQUIRE_VALUE(moved.save());
        CHECK(index_writes == 1);
    }
    CHECK(index_writes == 1);

    {
        store::PersistedIndex<Index> destination(Index {}, counting_index_format(), "second", false);
        REQUIRE(destination.add(root, store_test::int_hash(2)));
        store::PersistedIndex<Index> source(Index {}, counting_index_format(), "third", false);
        REQUIRE(source.add(root, store_test::int_hash(3)));
        destination = std::move(source);
        CHECK(index_writes == 2);
    }
    CHECK(index_writes == 3);
}
