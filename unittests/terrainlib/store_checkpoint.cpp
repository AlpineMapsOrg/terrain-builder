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

#include <filesystem>

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "octree/Id.h"
#include "store/Checkpoint.h"
#include "store/inputs.h"
#include "store_test_utils.h"

TEST_CASE("open_for_reuse opens a matching draft whose nodes can be linked", "[store][Checkpoint]")
{
    const TempDirectory directory("checkpoint-reuse");
    const store_test::Inputs inputs { "input", 1 };
    const octree::Id root = octree::Id::root();
    store_test::Draft earlier = REQUIRE_VALUE(store_test::create_draft(directory.path() / "earlier"));
    REQUIRE_VALUE(earlier.save(root, 41));
    REQUIRE_VALUE(earlier.save_index());
    REQUIRE_VALUE(store::inputs::write<store_test::InputsSchema>(earlier.base_path(), inputs));
    const std::filesystem::path output_path = directory.path() / "output";
    store_test::Draft output = REQUIRE_VALUE(store_test::create_draft(output_path));

    auto open_result = store::open_for_reuse<store_test::InputsSchema>(earlier.base_path(), output_path, inputs, store_test::open_storage);
    const store_test::Checkpoint checkpoint = REQUIRE_VALUE(std::move(open_result));

    CHECK(checkpoint.has(root));
    REQUIRE_VALUE(output.link_from(root, checkpoint));
    CHECK(REQUIRE_VALUE(output.load(root)) == 41);
}
