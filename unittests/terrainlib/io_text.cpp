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

#include <catch2/catch_test_macros.hpp>

#include <filesystem>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "io/text.h"

TEST_CASE("write and read round-trip text", "[io][text]") {
    const TempDirectory directory("io_text_round_trip");
    const std::filesystem::path path = directory.path() / "nested" / "file.txt";

    REQUIRE_VALUE(io::text::write("first line\nsecond line\n", path));

    CHECK(REQUIRE_VALUE(io::text::read(path)) == "first line\nsecond line\n");
}
