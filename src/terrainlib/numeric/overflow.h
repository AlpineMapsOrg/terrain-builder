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

#include <concepts>
#include <limits>
#include <type_traits>

#include "macros.h"

namespace detail {
template <std::integral T>
[[nodiscard]] constexpr bool add_overflow_fallback(const T lhs, const T rhs, T &result) noexcept {
    constexpr T minimum = std::numeric_limits<T>::min();
    constexpr T maximum = std::numeric_limits<T>::max();

    if constexpr (std::is_unsigned_v<T>) {
        if (rhs > maximum - lhs) {
            return true;
        }
    } else {
        if (rhs > T{0} && lhs > maximum - rhs) {
            return true;
        }

        if (rhs < T{0} && lhs < minimum - rhs) {
            return true;
        }
    }

    result = static_cast<T>(lhs + rhs);
    return false;
}

template <std::integral T>
[[nodiscard]] constexpr bool sub_overflow_fallback(const T lhs, const T rhs, T &result) noexcept {
    constexpr T minimum = std::numeric_limits<T>::min();
    constexpr T maximum = std::numeric_limits<T>::max();

    if constexpr (std::is_unsigned_v<T>) {
        if (lhs < rhs) {
            return true;
        }
    } else {
        if (rhs > T{0} && lhs < minimum + rhs) {
            return true;
        }

        if (rhs < T{0} && lhs > maximum + rhs) {
            return true;
        }
    }

    result = static_cast<T>(lhs - rhs);
    return false;
}

template <std::integral T>
[[nodiscard]] constexpr bool mul_overflow_fallback(const T lhs, const T rhs, T &result) noexcept {
    constexpr T minimum = std::numeric_limits<T>::min();
    constexpr T maximum = std::numeric_limits<T>::max();

    if constexpr (std::is_unsigned_v<T>) {
        if (lhs != T{0} && rhs > maximum / lhs) {
            return true;
        }
    } else {
        if (lhs > T{0} && rhs > T{0} && lhs > maximum / rhs) {
            return true;
        }

        if (lhs > T{0} && rhs < T{0} && rhs < minimum / lhs) {
            return true;
        }

        if (lhs < T{0} && rhs > T{0} && lhs < minimum / rhs) {
            return true;
        }

        if (lhs < T{0} && rhs < T{0} && lhs < maximum / rhs) {
            return true;
        }
    }

    result = static_cast<T>(lhs * rhs);
    return false;
}
}

template <std::integral T>
[[nodiscard]] constexpr bool add_overflow(const T lhs, const T rhs, T &result) noexcept {
#if ALP_HAS_BUILTIN(__builtin_add_overflow)
    if (!std::is_constant_evaluated()) {
        T temporary;

        if (__builtin_add_overflow(lhs, rhs, &temporary)) {
            return true;
        }

        result = temporary;
        return false;
    }
#endif

    return detail::add_overflow_fallback(lhs, rhs, result);
}

template <std::integral T>
[[nodiscard]]
constexpr bool sub_overflow(const T lhs, const T rhs, T &result) noexcept {
#if ALP_HAS_BUILTIN(__builtin_sub_overflow)
    if (!std::is_constant_evaluated()) {
        T temporary;

        if (__builtin_sub_overflow(lhs, rhs, &temporary)) {
            return true;
        }

        result = temporary;
        return false;
    }
#endif

    return detail::sub_overflow_fallback(lhs, rhs, result);
}

// Writes the product of `lhs` and `rhs` to `result` and returns false if it fits in T; otherwise returns true and leaves `result` unchanged.
template <std::integral T>
[[nodiscard]] constexpr bool mul_overflow(const T lhs, const T rhs, T &result) noexcept {
#if ALP_HAS_BUILTIN(__builtin_mul_overflow)
    if (!std::is_constant_evaluated()) {
        T temporary;

        if (__builtin_mul_overflow(lhs, rhs, &temporary)) {
            return true;
        }

        result = temporary;
        return false;
    }
#endif

    return detail::mul_overflow_fallback(lhs, rhs, result);
}
