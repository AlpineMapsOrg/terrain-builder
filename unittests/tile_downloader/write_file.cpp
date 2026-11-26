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

#include "write_file.h"

#include <filesystem>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "io/filesystem.h"
#include "io/text.h"

TEST_CASE("checked file writer persists the complete response") {
    const TempDirectory directory("atb-write-file-success");
    const std::filesystem::path path = directory.path() / "tile.bin";
    const std::filesystem::path partial_path = partial_tile_path(path);
    const std::filesystem::path pending_path = children_pending_tile_path(path);
    const std::string expected = "tile";

    write_file_children_pending(path, std::vector<char>(expected.begin(), expected.end()));

    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(path)));
    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(partial_path)));
    REQUIRE(REQUIRE_VALUE(io::filesystem::exists(pending_path)));

    CHECK(REQUIRE_VALUE(io::text::read(pending_path)) == expected);

    mark_tile_children_complete(path);
    CHECK(REQUIRE_VALUE(io::filesystem::exists(path)));
    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(pending_path)));
}

TEST_CASE("checked file writer preserves the final path when promotion fails") {
    const TempDirectory directory("atb-write-file-promotion-failure");
    const std::filesystem::path path = directory.path() / "tile";
    const std::filesystem::path pending_path = children_pending_tile_path(path);
    REQUIRE(REQUIRE_VALUE(io::filesystem::create_directory(path)));
    write_file_children_pending(path, std::vector<char>{'t', 'i', 'l', 'e'});

    CHECK_THROWS_AS(
        mark_tile_children_complete(path),
        std::filesystem::filesystem_error);

    CHECK(REQUIRE_VALUE(io::filesystem::is_directory(path)));
    CHECK(REQUIRE_VALUE(io::filesystem::exists(pending_path)));
}

#if defined(__linux__)
TEST_CASE("checked file writer reports persistence failures") {
    const std::vector<char> data(64 * 1024, 'x');

    CHECK_THROWS_AS(write_file_children_pending("/dev/full", data), std::runtime_error);
    CHECK(std::filesystem::is_character_file("/dev/full"));
    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists("/dev/full.part")));
    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists("/dev/full.children-pending")));
}
#endif
