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

#include <stdexcept>

#include <catch2/catch_test_macros.hpp>

// A failed assertion, as thrown in tests by the handler throwing_assertions.cpp installs.
struct AssertionFailure : std::logic_error {
    using std::logic_error::logic_error;
};

// Checks that evaluating the expression fails an assertion.
#define CHECK_ASSERTS(...) CHECK_THROWS_AS(__VA_ARGS__, AssertionFailure)

// Requires that evaluating the expression fails an assertion.
#define REQUIRE_ASSERTS(...) REQUIRE_THROWS_AS(__VA_ARGS__, AssertionFailure)
