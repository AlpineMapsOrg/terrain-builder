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

#include "numeric/overflow.h"

template <std::integral T>
[[nodiscard]] constexpr T saturating_add(const T lhs, const T rhs) noexcept {
    T result;

    if (!add_overflow(lhs, rhs, result)) {
        return result;
    }

    if constexpr (std::is_unsigned_v<T>) {
        return std::numeric_limits<T>::max();
    } else {
        return rhs > T{0} ? std::numeric_limits<T>::max() : std::numeric_limits<T>::min();
    }
}

template <std::integral T>
[[nodiscard]] constexpr T saturating_sub(const T lhs, const T rhs) noexcept {
    T result;

    if (!sub_overflow(lhs, rhs, result)) {
        return result;
    }

    if constexpr (std::is_unsigned_v<T>) {
        return T{0};
    } else {
        return rhs > T{0} ? std::numeric_limits<T>::min() : std::numeric_limits<T>::max();
    }
}

template <std::integral T>
[[nodiscard]] constexpr T saturating_mul(const T lhs, const T rhs) noexcept {
    T result;

    if (!mul_overflow(lhs, rhs, result)) {
        return result;
    }

    if constexpr (std::is_unsigned_v<T>) {
        return std::numeric_limits<T>::max();
    } else {
        return (lhs < T{0}) == (rhs < T{0}) ? std::numeric_limits<T>::max() : std::numeric_limits<T>::min();
    }
}
