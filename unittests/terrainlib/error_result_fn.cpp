#include <catch2/catch_test_macros.hpp>

#include <concepts>
#include <string_view>

#include "error/Error.h"
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
    alp::ResultFn<WideError, decltype(add)> fn(add);

    STATIC_REQUIRE(std::same_as<decltype(fn(3, 4)), alp::Result<void, WideError>>);
    CHECK(fn(3, 4).has_value());
    CHECK(sum == 7);
}

TEST_CASE("ResultFn wraps a returned value in a succeeded result", "[error][ResultFn]") {
    const auto add = [](const int a, const int b) {
        return a + b;
    };
    alp::ResultFn<WideError, decltype(add)> fn(add);

    STATIC_REQUIRE(std::same_as<decltype(fn(3, 4)), alp::Result<int, WideError>>);
    CHECK(fn(3, 4) == 7);
}

TEST_CASE("ResultFn keeps the value of a succeeded result", "[error][ResultFn]") {
    const auto add = [](const int a, const int b) -> alp::Result<int, ParseError> {
        return a + b;
    };
    alp::ResultFn<WideError, decltype(add)> fn(add);

    STATIC_REQUIRE(std::same_as<decltype(fn(3, 4)), alp::Result<int, WideError>>);
    CHECK(fn(3, 4) == 7);
}

TEST_CASE("ResultFn converts a failed result into the wider error, keeping its code", "[error][ResultFn]") {
    SECTION("with a value") {
        const auto parse = []() -> alp::Result<int, ParseError> {
            return alp::make_unexpected(ParseError::Malformed);
        };
        alp::ResultFn<WideError, decltype(parse)> fn(parse);

        const alp::Result<int, WideError> result = fn();
        REQUIRE_FALSE(result.has_value());
        CHECK(result.error().holds(ParseError::Malformed));
    }

    SECTION("without a value") {
        const auto parse = []() -> alp::Result<void, ParseError> {
            return alp::make_unexpected(ParseError::Malformed);
        };
        alp::ResultFn<WideError, decltype(parse)> fn(parse);

        const alp::Result<void, WideError> result = fn();
        REQUIRE_FALSE(result.has_value());
        CHECK(result.error().holds(ParseError::Malformed));
    }
}
