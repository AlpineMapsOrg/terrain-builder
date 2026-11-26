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

#include <cmath>
#include <concepts>
#include <limits>
#include <optional>
#include <type_traits>
#include <utility>

#include <glm/glm.hpp>
#include <libassert/assert.hpp>

#include "glm_utils.h"

// An integral or floating-point type.
template <typename T>
concept Arithmetic = std::is_arithmetic_v<T>;

// Whether `value` converts to `To` without changing.
template <Arithmetic To, Arithmetic From>
[[nodiscard]] constexpr bool is_representable(const From value) {
    if constexpr (std::integral<To> && std::integral<From>) {
        return std::in_range<To>(value);
    } else if constexpr (std::integral<To>) {
        // To's maximum can round up in From, but 2^digits is exact.
        const From upper_bound = std::ldexp(From{1}, std::numeric_limits<To>::digits);
        const bool in_range = value >= static_cast<From>(std::numeric_limits<To>::min()) && value < upper_bound;
        return in_range && std::trunc(value) == value;
    } else if constexpr (std::integral<From>) {
        // Rounding can carry a value up to 2^digits, which From cannot hold.
        const To converted = static_cast<To>(value);
        const To upper_bound = std::ldexp(To{1}, std::numeric_limits<From>::digits);
        return converted < upper_bound && static_cast<From>(converted) == value;
    } else {
        // Every floating-point type has NaN and the infinities.
        if (std::isnan(value) || std::isinf(value)) {
            return true;
        }
        // Converting a finite value beyond To's range is undefined.
        if (std::abs(value) > std::numeric_limits<To>::max()) {
            return false;
        }
        return static_cast<From>(static_cast<To>(value)) == value;
    }
}

// Converts `value` to `To`, or nothing if `value` is not representable in `To`.
template <Arithmetic To, Arithmetic From>
[[nodiscard]] constexpr std::optional<To> try_cast(const From value) {
    if (is_representable<To>(value)) {
        return static_cast<To>(value);
    } else {
        return std::nullopt;
    }
}

// Converts `value` to `To`.
// Asserts that `value` is representable in `To`.
template <Arithmetic To, Arithmetic From>
[[nodiscard]] inline constexpr To checked_cast(const From value) {
    ASSERT(is_representable<To>(value));
    return static_cast<To>(value);
}

// Converts `value` to `To`.
// Asserts in debug builds that `value` is representable in `To`.
template <Arithmetic To, Arithmetic From>
[[nodiscard]] inline constexpr To debug_checked_cast(const From value) {
    DEBUG_ASSERT(is_representable<To>(value));
    return static_cast<To>(value);
}

// Converts each component of `v` to `To`.
// Asserts that every component is representable in `To`.
template <Arithmetic To, glm::length_t L, Arithmetic From, glm::qualifier Q>
[[nodiscard]] inline constexpr glm::vec<L, To, Q> checked_cast(const glm::vec<L, From, Q> &v) {
    return transform(v, [](const From c) {
        return checked_cast<To>(c);
    });
}

// Converts each component of `v` to `To`.
// Asserts in debug builds that every component is representable in `To`.
template <Arithmetic To, glm::length_t L, Arithmetic From, glm::qualifier Q>
[[nodiscard]] inline constexpr glm::vec<L, To, Q> debug_checked_cast(const glm::vec<L, From, Q> &v) {
    return transform(v, [](const From c) {
        return debug_checked_cast<To>(c);
    });
}
