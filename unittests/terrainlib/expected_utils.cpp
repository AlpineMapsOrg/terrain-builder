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

#include <expected>
#include <memory>
#include <string>
#include <utility>

#include <catch2/catch_test_macros.hpp>

#include "expected_utils.h"

TEST_CASE("value_or_else gives the value of a result that holds one", "[expected_utils]") {
    const std::expected<size_t, std::string> result = 3;
    CHECK(alp::value_or_else(result, [](const std::string &error) { return error.size(); }) == 3);
}

TEST_CASE("value_or_else gives what its function makes from the error of a result that holds one", "[expected_utils]") {
    const std::expected<size_t, std::string> result = std::unexpected(std::string("four"));
    CHECK(alp::value_or_else(result, [](const std::string &error) { return error.size(); }) == 4);
}

TEST_CASE("value_or_else moves the error into its function", "[expected_utils]") {
    std::expected<int, std::unique_ptr<int>> result = std::unexpected(std::make_unique<int>(5));
    CHECK(alp::value_or_else(std::move(result), [](std::unique_ptr<int> error) { return *error; }) == 5);
}
