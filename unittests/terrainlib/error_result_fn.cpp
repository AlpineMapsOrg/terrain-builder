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

#include <concepts>
#include <string_view>

#include "error/Error.h"
#include "error/Infallible.h"
#include "error/Result.h"
#include "error/ResultFn.h"
#include "error/utils.h"

namespace {

enum class ReadError {
    Missing,
};

std::string_view format_as(const ReadError) {
    return "missing";
}

enum class ParseError {
    Malformed,
};

std::string_view format_as(const ParseError) {
    return "malformed";
}

// An error wider than the one the callables below fail with.
using WideError = alp::Error<ReadError, ParseError>;

} // namespace

TEST_CASE("ResultFn turns a callable returning nothing into a succeeded result without a value", "[error][ResultFn]") {
    int sum = 0;
    const auto add = [&](const int a, const int b) {
        sum = a + b;
    };
    auto fn = alp::make_result_fn<WideError>(add);

    STATIC_REQUIRE(std::same_as<decltype(fn(3, 4)), alp::Result<void, WideError>>);
    CHECK(fn(3, 4).has_value());
    CHECK(sum == 7);
}

TEST_CASE("ResultFn wraps a returned value in a succeeded result", "[error][ResultFn]") {
    const auto add = [](const int a, const int b) {
        return a + b;
    };
    auto fn = alp::make_result_fn<WideError>(add);

    STATIC_REQUIRE(std::same_as<decltype(fn(3, 4)), alp::Result<int, WideError>>);
    CHECK(fn(3, 4) == 7);
}

TEST_CASE("ResultFn keeps the value of a succeeded result", "[error][ResultFn]") {
    const auto add = [](const int a, const int b) -> alp::Result<int, ParseError> {
        return a + b;
    };
    auto fn = alp::make_result_fn<WideError>(add);

    STATIC_REQUIRE(std::same_as<decltype(fn(3, 4)), alp::Result<int, WideError>>);
    CHECK(fn(3, 4) == 7);
}

TEST_CASE("ResultFn converts a failed result into the wider error, keeping its code", "[error][ResultFn]") {
    SECTION("with a value") {
        const auto parse = []() -> alp::Result<int, ParseError> {
            return alp::make_error(ParseError::Malformed);
        };
        auto fn = alp::make_result_fn<WideError>(parse);

        const alp::Result<int, WideError> result = fn();
        REQUIRE_FALSE(result.has_value());
        CHECK(result.error().holds_code(ParseError::Malformed));
    }

    SECTION("without a value") {
        const auto parse = []() -> alp::Result<void, ParseError> {
            return alp::make_error(ParseError::Malformed);
        };
        auto fn = alp::make_result_fn<WideError>(parse);

        const alp::Result<void, WideError> result = fn();
        REQUIRE_FALSE(result.has_value());
        CHECK(result.error().holds_code(ParseError::Malformed));
    }
}

TEST_CASE("invoke_fallible wraps a returned value in a succeeded result with the wider error", "[error][ResultFn]") {
    const auto add = [](const int a, const int b) {
        return a + b;
    };

    STATIC_REQUIRE(std::same_as<decltype(alp::invoke_fallible<WideError>(add, 3, 4)), alp::Result<int, WideError>>);
    CHECK(alp::invoke_fallible<WideError>(add, 3, 4) == 7);
}

TEST_CASE("invoke_fallible converts a failed result into the wider error, keeping its code", "[error][ResultFn]") {
    const auto parse = [](const int) -> alp::Result<int, ParseError> {
        return alp::make_error(ParseError::Malformed);
    };

    const alp::Result<int, WideError> result = alp::invoke_fallible<WideError>(parse, 3);
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().holds_code(ParseError::Malformed));
}

TEST_CASE("invoke_fallible without codes keeps the callable's error", "[error][ResultFn]") {
    const auto ignore = [](int) {};
    const auto add = [](const int a, const int b) {
        return a + b;
    };
    const auto parse = [](const int) -> alp::Result<int, ParseError> {
        return 7;
    };
    const auto parse_any = [](const int) -> alp::ErasedResult<int> {
        return 7;
    };

    STATIC_REQUIRE(std::same_as<decltype(alp::invoke_fallible(ignore, 3)), alp::Result<void, alp::Infallible>>);
    STATIC_REQUIRE(std::same_as<decltype(alp::invoke_fallible(add, 3, 4)), alp::Result<int, alp::Infallible>>);
    STATIC_REQUIRE(std::same_as<decltype(alp::invoke_fallible(parse, 3)), alp::Result<int, ParseError>>);
    STATIC_REQUIRE(std::same_as<decltype(alp::invoke_fallible(parse_any, 3)), alp::ErasedResult<int>>);
}
