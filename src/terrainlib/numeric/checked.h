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
#include <optional>

#include "numeric/overflow.h"

// The sum of `lhs` and `rhs`, if it fits in T.
template <std::integral T>
[[nodiscard]] constexpr std::optional<T> checked_add(const T lhs, const T rhs) noexcept {
    T result;

    if (add_overflow(lhs, rhs, result)) {
        return std::nullopt;
    }

    return result;
}

// The difference of `lhs` and `rhs`, if it fits in T.
template <std::integral T>
[[nodiscard]] constexpr std::optional<T> checked_sub(const T lhs, const T rhs) noexcept {
    T result;

    if (sub_overflow(lhs, rhs, result)) {
        return std::nullopt;
    }

    return result;
}

// The product of `lhs` and `rhs`, if it fits in T.
template <std::integral T>
[[nodiscard]] constexpr std::optional<T> checked_mul(const T lhs, const T rhs) noexcept {
    T result;

    if (mul_overflow(lhs, rhs, result)) {
        return std::nullopt;
    }

    return result;
}
