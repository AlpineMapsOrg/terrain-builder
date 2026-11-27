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

#pragma once

#include <utility>

#include <catch2/catch_test_macros.hpp>
#include <fmt/format.h>

#include "error/format.h"
#include "macros.h"

// REQUIRE_VALUE(expr) with its result variable spelled out.
#define REQUIRE_VALUE_IMPL(result, ...)                                             \
    [&] {                                                                           \
        auto result = (__VA_ARGS__);                                                \
        if (!result.has_value()) {                                                  \
            FAIL(::fmt::format("the result holds an error\n{:?}", result.error())); \
        }                                                                           \
        return std::move(result).value();                                           \
    }()

// The value of `result`; fails the test with the error's report if it holds none.
#define REQUIRE_VALUE(...) REQUIRE_VALUE_IMPL(ALP_UNIQUE_IDENT(_require_value_result_), __VA_ARGS__)

// REQUIRE_ERROR(expr) with its result variable spelled out.
#define REQUIRE_ERROR_IMPL(result, ...)       \
    [&] {                                     \
        auto result = (__VA_ARGS__);          \
        if (result.has_value()) {             \
            FAIL("the result holds a value"); \
        }                                     \
        return std::move(result).error();     \
    }()

// The error of `result`; fails the test if it holds a value.
#define REQUIRE_ERROR(...) REQUIRE_ERROR_IMPL(ALP_UNIQUE_IDENT(_require_error_result_), __VA_ARGS__)
