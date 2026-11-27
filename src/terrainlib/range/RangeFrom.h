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

#include "Bound.h"
#include "RangeBounds.h"
#include "numeric/checked_cast.h"

template <typename T>
struct RangeFrom : RangeBounds<RangeFrom<T>, T> {
    T start;

    constexpr RangeFrom(T start_value) : start(start_value) {}

    template <typename U>
    explicit constexpr RangeFrom(const RangeFrom<U> &other) : RangeFrom(debug_checked_cast<T>(other.start)) {}

    [[nodiscard]] constexpr Bound<T> start_bound() const noexcept {
        return Bound<T>::included(this->start);
    }

    [[nodiscard]] constexpr Bound<T> end_bound() const noexcept {
        return Bound<T>::unbounded();
    }

    [[nodiscard]] constexpr bool operator==(const RangeFrom &) const noexcept = default;
};
