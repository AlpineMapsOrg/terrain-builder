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

#include <optional>

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "octree/Id.h"
#include "octree/storage/Index.h"
#include "octree/storage/index_format.h"
#include "store/Index.h"
#include "store/open.h"
#include "store_test_utils.h"

TEST_CASE("open rejects indexes that lack required hashes", "[store][open]")
{
    const TempDirectory directory("open-hash-required");
    octree::storage::Index index;
    REQUIRE(index.add(octree::Id::root(), std::nullopt));
    REQUIRE_VALUE(octree::storage::write_index(directory.path(), index));

    CHECK(REQUIRE_ERROR(store_test::open_storage(directory.path(), store::Accept::Published)).holds<store::CorruptIndex>());
}
