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
#include <system_error>

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "io/FileError.h"
#include "io/bytes.h"
#include "io/filesystem.h"
#include "store/create.h"
#include "store/inputs.h"
#include "store/publish.h"
#include "store/storage_paths.h"
#include "store_test_utils.h"

TEST_CASE("drafts are created as .part and published by renaming", "[store][create][publish]")
{
    const TempDirectory directory("draft-publish");
    const store_test::Inputs inputs { "input", 1 };
    const std::filesystem::path final_path = directory.path() / "nested" / "output";
    const std::filesystem::path draft_path = store::make_draft_path(final_path);
    CHECK(draft_path.filename() == "output.part");
    CHECK(store::is_draft(draft_path));
    CHECK_FALSE(store::is_draft(final_path));

    store_test::Draft draft = REQUIRE_VALUE(store_test::create_draft(final_path));
    CHECK(draft.base_path() == draft_path);
    CHECK(REQUIRE_VALUE(io::filesystem::is_directory(draft_path)));

    SECTION("an existing .part is rejected")
    {
        const auto error = REQUIRE_ERROR(store_test::create_draft(final_path));
        CHECK(error.holds_code(io::FileError(std::errc::file_exists, draft_path)));
    }

    SECTION("a .part final path is rejected")
    {
        CHECK(REQUIRE_ERROR(store_test::create_draft(draft_path)).holds<store::InvalidStoragePath>());
    }

    SECTION("publication removes the input record and renames")
    {
        REQUIRE_VALUE(store::inputs::write<store_test::InputsSchema>(draft_path, inputs));
        REQUIRE(REQUIRE_VALUE(io::filesystem::exists(draft_path / store::inputs::file_name)));
        REQUIRE_VALUE(store::publish(std::move(draft)));
        CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(draft_path)));
        CHECK(REQUIRE_VALUE(io::filesystem::is_directory(final_path)));
        CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(final_path / store::inputs::file_name)));

        const auto error = REQUIRE_ERROR(store_test::create_draft(final_path));
        CHECK(error.holds_code(io::FileError(std::errc::file_exists, final_path)));
    }

    SECTION("publication does not replace an existing destination")
    {
        SECTION("empty directory") { REQUIRE(REQUIRE_VALUE(io::filesystem::create_directory(final_path))); }
        SECTION("nonempty directory") { REQUIRE_VALUE(io::bytes::write(io::bytes::vector {}, final_path / "keep")); }
        SECTION("dangling symlink") { REQUIRE_VALUE(io::filesystem::create_symlink(directory.path() / "missing", final_path)); }
        REQUIRE_VALUE(store::inputs::write<store_test::InputsSchema>(draft_path, inputs));

        const auto error = REQUIRE_ERROR(store::publish(std::move(draft)));
        CHECK(error.holds_code(io::FileError(std::errc::file_exists, final_path)));
        // The retained .part stays usable as a checkpoint.
        REQUIRE_VALUE(store::inputs::validate_reuse<store_test::InputsSchema>(draft_path, final_path, inputs));
    }
}
