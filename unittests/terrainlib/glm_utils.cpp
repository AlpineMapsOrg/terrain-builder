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

#include <limits>
#include <type_traits>

#include <catch2/catch_test_macros.hpp>
#include <glm/glm.hpp>

#include "glm_utils.h"

TEST_CASE("is_finite holds for a vector of finite components", "[glm_utils]") {
    CHECK(is_finite(glm::dvec3(1, -2, 0)));
}

TEST_CASE("is_finite fails for a vector with a non-finite component", "[glm_utils]") {
    SECTION("infinite") {
        CHECK_FALSE(is_finite(glm::dvec2(std::numeric_limits<double>::infinity(), 0)));
    }
    SECTION("NaN") {
        CHECK_FALSE(is_finite(glm::dvec2(0, std::numeric_limits<double>::quiet_NaN())));
    }
}

TEST_CASE("transform applies an operation to every component", "[glm_utils]") {
    const glm::ivec3 v = {1, 2, 3};

    CHECK(transform(v, [](const auto& c) { return c * 2; }) == glm::ivec3{2, 4, 6});
}

TEST_CASE("transform changes the component type to the operation's result", "[glm_utils]") {
    const glm::ivec2 v = {1, 2};
    const auto halved = transform(v, [](const auto& c) { return c / 2.0; });

    STATIC_CHECK(std::is_same_v<decltype(halved), const glm::dvec2>);
    CHECK(halved == glm::dvec2{0.5, 1.0});
}

TEST_CASE("transform pairs components of two vectors", "[glm_utils]") {
    const glm::ivec3 lhs = {1, 2, 3};
    const glm::ivec3 rhs = {10, 20, 30};

    CHECK(transform(lhs, rhs, [](const auto& a, const auto& b) { return a + b; }) == glm::ivec3{11, 22, 33});
}

TEST_CASE("transform pairs components of differently typed vectors", "[glm_utils]") {
    const glm::ivec2 lhs = {1, 2};
    const glm::dvec2 rhs = {0.5, 0.25};
    const auto scaled = transform(lhs, rhs, [](const auto& a, const auto& b) { return a * b; });

    STATIC_CHECK(std::is_same_v<decltype(scaled), const glm::dvec2>);
    CHECK(scaled == glm::dvec2{0.5, 0.5});
}
