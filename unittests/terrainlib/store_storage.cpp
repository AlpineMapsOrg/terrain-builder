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

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "io/filesystem.h"
#include "octree/Id.h"
#include "store/Hash.h"
#include "store/Storage.h"
#include "store/open.h"
#include "store/storage_paths.h"
#include "store_test_utils.h"

TEST_CASE("hash reads the recorded hashes without node-file access", "[store][Storage]")
{
    const TempDirectory directory("storage-hash");
    const octree::Id root = octree::Id::root();
    const octree::Id child = root.child(2).value();
    const std::filesystem::path final_path = directory.path() / "storage";
    {
        store_test::Draft draft = REQUIRE_VALUE(store_test::create_draft(final_path));
        REQUIRE_VALUE(draft.save(root, 0));
        REQUIRE_VALUE(draft.save(child, 7));
        // The all-zero hash of zero is an identity like any other.
        CHECK(REQUIRE_VALUE(draft.hash(root)) == store::PayloadHash(sizeof(int), 0));
        CHECK(REQUIRE_VALUE(draft.hash(child)) == store_test::int_hash(7));
        REQUIRE_VALUE(draft.save_index());
    }

    const store_test::Storage storage = REQUIRE_VALUE(store_test::open_storage(store::make_draft_path(final_path), store::Accept::Draft));
    CHECK(REQUIRE_VALUE(storage.load(child)) == 7);
    REQUIRE(REQUIRE_VALUE(io::filesystem::remove(storage.paths(child).front())));
    CHECK(REQUIRE_VALUE(storage.hash(child)) == store_test::int_hash(7));
    CHECK(REQUIRE_VALUE(storage.hash(root)) == store::PayloadHash(sizeof(int), 0));
}
