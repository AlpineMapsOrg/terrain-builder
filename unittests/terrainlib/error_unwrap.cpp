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

#include <stdexcept>
#include <string_view>

#include "error/Error.h"
#include "error/Result.h"
#include "error/unwrap.h"
#include "error/utils.h"

namespace {

enum class FileError {
    NotFound,
};

std::string_view format_as(const FileError) {
    return "file not found";
}

} // namespace

TEST_CASE("unwrap yields the value of a result that holds one", "[error][unwrap]") {
    const alp::Result<int, FileError> result = 7;

    SECTION("asserting") {
        CHECK(alp::unwrap(result) == 7);
    }
    SECTION("throwing") {
        CHECK(alp::throwing_unwrap(result) == 7);
    }
}

TEST_CASE("unwrap returns for a void result that succeeded", "[error][unwrap]") {
    const alp::Result<void, FileError> result;

    SECTION("asserting") {
        CHECK_NOTHROW(alp::unwrap(result));
    }
    SECTION("throwing") {
        CHECK_NOTHROW(alp::throwing_unwrap(result));
    }
}

TEST_CASE("unwrap fails for a result that holds an error", "[error][unwrap]") {
    const alp::Result<int, FileError> result = std::unexpected(alp::make_error(FileError::NotFound));

    SECTION("asserting") {
        // In tests a failed assertion throws a std::logic_error instead of aborting; see unittests/throwing_assertions.cpp.
        CHECK_THROWS_AS(alp::unwrap(result), std::logic_error);
    }
    SECTION("throwing") {
        CHECK_THROWS_AS(alp::throwing_unwrap(result), alp::UnwrapError);
    }
}

TEST_CASE("expect yields the value of a result that holds one", "[error][unwrap]") {
    const alp::Result<int, FileError> result = 7;

    SECTION("asserting") {
        CHECK(alp::expect(result, "reading the answer") == 7);
    }
    SECTION("throwing") {
        CHECK(alp::throwing_expect(result, "reading the answer") == 7);
    }
}

TEST_CASE("expect fails for a result that holds an error", "[error][unwrap]") {
    const alp::Result<int, FileError> result = std::unexpected(alp::make_error(FileError::NotFound));

    SECTION("asserting") {
        CHECK_THROWS_AS(alp::expect(result, "reading the answer"), std::logic_error);
    }
    SECTION("throwing") {
        CHECK_THROWS_AS(alp::throwing_expect(result, "reading the answer"), alp::UnwrapError);
    }
}
