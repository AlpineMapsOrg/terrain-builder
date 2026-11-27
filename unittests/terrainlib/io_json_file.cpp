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
#include <catch2/generators/catch_generators.hpp>

#include <filesystem>
#include <limits>
#include <string>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "common_error_codes.h"
#include "error/prelude.h"
#include "io/FileError.h"
#include "io/filesystem.h"
#include "io/json/file.h"
#include "io/text.h"

namespace json = io::json;

TEST_CASE("write writes what to_string produces for the given format", "[io][json][file]") {
    const TempDirectory directory("json");
    const std::filesystem::path path = directory.path() / "value.json";
    const json::Value value = json::Object{{"a", json::Array{1}}};
    const json::Format format = GENERATE(json::Format::Compact, json::Format::Pretty);

    REQUIRE_VALUE(json::write(value, path, format));

    const std::string expected = REQUIRE_VALUE(json::to_string(value, format));
    CHECK(REQUIRE_VALUE(io::text::read(path)) == expected);
}

TEST_CASE("write passes the write options on", "[io][json][file]") {
    const TempDirectory directory("json");
    const std::filesystem::path path = directory.path() / "missing" / "value.json";

    const auto error = REQUIRE_ERROR(json::write(json::Value(1), path, json::Format::Compact, {.make_dirs = false}));

    CHECK(error.holds<io::FileError>());
}

TEST_CASE("write rejects a value to_string rejects and creates no file", "[io][json][file]") {
    const TempDirectory directory("json");
    const std::filesystem::path path = directory.path() / "value.json";

    const auto error = REQUIRE_ERROR(json::write(json::Value(std::numeric_limits<double>::quiet_NaN()), path));

    CHECK(error.holds<Unsupported>());
    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(path)));
}
