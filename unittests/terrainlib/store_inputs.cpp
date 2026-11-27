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
#include "store/inputs.h"
#include "store_test_utils.h"

TEST_CASE("validate_reuse requires a draft with matching inputs", "[store][inputs]")
{
    const TempDirectory directory("inputs-reuse");
    const store_test::Inputs inputs { "input", 1 };
    const std::filesystem::path output_path = directory.path() / "output";
    const std::filesystem::path draft_path = directory.path() / "output.part";
    REQUIRE(REQUIRE_VALUE(io::filesystem::create_directory(draft_path)));

    SECTION("missing inputs")
    {
        const auto error = REQUIRE_ERROR(store::inputs::validate_reuse<store_test::InputsSchema>(draft_path, output_path, inputs));
        CHECK(error.holds_code(store::inputs::ReuseError::MissingInputs));
    }

    SECTION("matching inputs")
    {
        REQUIRE_VALUE(store::inputs::write<store_test::InputsSchema>(draft_path, inputs));
        REQUIRE_VALUE(store::inputs::validate_reuse<store_test::InputsSchema>(draft_path, output_path, inputs));
        REQUIRE_VALUE(store::inputs::validate_reuse<store_test::InputsSchema>(draft_path.string() + "/", output_path, inputs));
    }

    SECTION("different inputs")
    {
        REQUIRE_VALUE(store::inputs::write<store_test::InputsSchema>(draft_path, inputs));
        const auto error = REQUIRE_ERROR(store::inputs::validate_reuse<store_test::InputsSchema>(draft_path, output_path, store_test::Inputs { "input", 2 }));
        CHECK(error.holds_code(store::inputs::ReuseError::DifferentInputs));
    }

    SECTION("not a draft")
    {
        const std::filesystem::path published_path = directory.path() / "published";
        REQUIRE(REQUIRE_VALUE(io::filesystem::create_directory(published_path)));
        REQUIRE_VALUE(store::inputs::write<store_test::InputsSchema>(published_path, inputs));
        const auto error = REQUIRE_ERROR(store::inputs::validate_reuse<store_test::InputsSchema>(published_path, output_path, inputs));
        CHECK(error.holds_code(store::inputs::ReuseError::NotDraft));
    }
}
