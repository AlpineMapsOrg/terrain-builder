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

#include <string>

#include "error/FallibleFn.h"
#include "error/Result.h"

TEST_CASE("FallibleFn accepts a callable that returns a result holding the value type", "[error][FallibleFn]") {
    const auto save = [](int) -> alp::ErasedResult<void> { return {}; };
    const auto parse = [](int) -> alp::ErasedResult<int> { return 1; };

    STATIC_REQUIRE(alp::FallibleFn<decltype(save), void, int>);
    STATIC_REQUIRE(alp::FallibleFn<decltype(parse), int, int>);
}

TEST_CASE("FallibleFn rejects a callable that returns a result holding another type", "[error][FallibleFn]") {
    const auto save = [](int) -> alp::ErasedResult<void> { return {}; };
    const auto parse = [](int) -> alp::ErasedResult<int> { return 1; };

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
    const auto save = [](int) -> alp::ErasedResult<void> { return {}; };

    STATIC_REQUIRE_FALSE(alp::FallibleFn<decltype(save), void, std::string>);
    STATIC_REQUIRE_FALSE(alp::FallibleFn<decltype(save), void>);
}
