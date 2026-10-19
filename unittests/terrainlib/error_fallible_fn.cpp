#include <catch2/catch_test_macros.hpp>

#include <string>

#include "error/FallibleFn.h"
#include "error/Result.h"

TEST_CASE("FallibleFn accepts a callable that returns a result holding the value type", "[error][FallibleFn]") {
    const auto save = [](int) -> alp::Result<void> { return {}; };
    const auto parse = [](int) -> alp::Result<int> { return 1; };

    STATIC_REQUIRE(alp::FallibleFn<decltype(save), void, int>);
    STATIC_REQUIRE(alp::FallibleFn<decltype(parse), int, int>);
}

TEST_CASE("FallibleFn rejects a callable that returns a result holding another type", "[error][FallibleFn]") {
    const auto save = [](int) -> alp::Result<void> { return {}; };
    const auto parse = [](int) -> alp::Result<int> { return 1; };

    STATIC_REQUIRE_FALSE(alp::FallibleFn<decltype(save), int, int>);
    STATIC_REQUIRE_FALSE(alp::FallibleFn<decltype(parse), void, int>);
}

TEST_CASE("FallibleFn rejects a callable that returns no result", "[error][FallibleFn]") {
    const auto log = [](int) {};
    const auto increment = [](int value) { return value + 1; };

    STATIC_REQUIRE_FALSE(alp::FallibleFn<decltype(log), void, int>);
    STATIC_REQUIRE_FALSE(alp::FallibleFn<decltype(increment), int, int>);
}

TEST_CASE("FallibleFn rejects a callable that does not take the arguments", "[error][FallibleFn]") {
    const auto save = [](int) -> alp::Result<void> { return {}; };

    STATIC_REQUIRE_FALSE(alp::FallibleFn<decltype(save), void, std::string>);
    STATIC_REQUIRE_FALSE(alp::FallibleFn<decltype(save), void>);
}
