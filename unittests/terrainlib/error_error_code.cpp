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

#include <cstdint>
#include <string>
#include <string_view>

#include <fmt/format.h>

#include "error/Error.h"
#include "error/ErrorCode.h"

namespace {

enum class FileError {
    NotFound,
};

std::string_view format_as(const FileError) {
    return "file not found";
}

// A code that carries data and does not compare.
struct OffsetError {
    uint32_t offset;
};

std::string format_as(const OffsetError error) {
    return fmt::format("bad byte at {}", error.offset);
}

// A code that also converts to a string view.
struct NamedError {
    std::string_view name;

    operator std::string_view() const {
        return this->name;
    }
};

std::string_view format_as(const NamedError error) {
    return error.name;
}

} // namespace

TEST_CASE("Error codes are unqualified formattable enums and classes other than strings", "[error][ErrorCode]") {
    enum class Unformatted {
        Broken,
    };

    STATIC_REQUIRE(alp::ErrorCode<FileError>);
    STATIC_REQUIRE(alp::ErrorCode<OffsetError>);
    STATIC_REQUIRE(alp::ErrorCode<NamedError>);
    STATIC_REQUIRE(!alp::ErrorCode<Unformatted>);
    STATIC_REQUIRE(!alp::ErrorCode<int>);
    STATIC_REQUIRE(!alp::ErrorCode<const char *>);
    STATIC_REQUIRE(!alp::ErrorCode<std::string>);
    STATIC_REQUIRE(!alp::ErrorCode<std::string_view>);
    STATIC_REQUIRE(!alp::ErrorCode<const FileError>);
    STATIC_REQUIRE(!alp::ErrorCode<FileError &>);
}

TEST_CASE("A typed error stands for its codes, but is not a code itself", "[error][ErrorCode]") {
    STATIC_REQUIRE(!alp::ErrorCode<alp::Error<FileError>>);
    STATIC_REQUIRE(alp::ErrorCodes<alp::Error<FileError>>);
    STATIC_REQUIRE(!alp::ErrorCodes<alp::ErasedError>);
}
