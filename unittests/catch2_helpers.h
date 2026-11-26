/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2022 Adam Celarek-Litofcenko
 * Copyright (C) 2022 Martin Braunsperger
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

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_tostring.hpp>
#include <glm/glm.hpp>
#include <radix/geometry.h>
#include <fmt/format.h>

#include "fmt_impls.h"
#include "geometry/geometry.h"
#include "type_name.h"

namespace Catch {
template <glm::length_t N, typename T>
struct StringMaker<glm::vec<N, T>> {
    static std::string convert(const glm::vec<N, T> &value) {
        return fmt::format("{}", value);
    }
};

template <glm::length_t N, typename T>
struct StringMaker<radix::geometry::Aabb<N, T>> {
    static std::string convert(const radix::geometry::Aabb<N, T> &value) {
        return fmt::format("Aabb{}{}({} - {})", N, type_name<T>()[0], value.min, value.max);
    }
};
}

// Matches a value equal to `target` up to `geometry::epsilon<double>()`, scaled by its magnitude above 1.
inline Catch::Matchers::WithinAbsMatcher WithinEpsilon(const double target) {
    return Catch::Matchers::WithinAbs(target, geometry::epsilon<double>() * std::max(1.0, std::abs(target)));
}

namespace detail {
inline bool is_within_epsilon(const double actual, const double expected) {
    return WithinEpsilon(expected).match(actual);
}

template <glm::length_t N>
bool is_within_epsilon(const glm::vec<N, double> &actual, const glm::vec<N, double> &expected) {
    for (glm::length_t i = 0; i < N; i++) {
        if (!is_within_epsilon(actual[i], expected[i])) {
            return false;
        }
    }
    return true;
}

template <glm::length_t N>
bool is_within_epsilon(const radix::geometry::Aabb<N, double> &actual, const radix::geometry::Aabb<N, double> &expected) {
    return is_within_epsilon(actual.min, expected.min) && is_within_epsilon(actual.max, expected.max);
}
} // namespace detail

// Matches a range holding the elements of `expected` in any order, each equal to its counterpart up to WithinEpsilon in every coordinate.
template <typename T>
auto UnorderedWithinEpsilon(std::vector<T> expected) {
    return Catch::Matchers::UnorderedRangeEquals(std::move(expected), [](const T &a, const T &b) { return detail::is_within_epsilon(a, b); });
}
