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

#include <algorithm>
#include <cmath>
#include <numbers>

#include <libassert/assert.hpp>

#include "geometry/geometry.h"

namespace gis::srs {

// The quantity a unit measures.
enum class Quantity {
    Length,
    Angle,
};

// A unit of measure.
struct Unit {
    Quantity quantity;
    // Metres for a length, radians for an angle.
    double si_size;

    bool operator==(const Unit &) const = default;

    // Whether `other` measures the same quantity and differs in size by at most `relative_tolerance`.
    [[nodiscard]]
    bool is_approx(const Unit &other, const double relative_tolerance = geometry::epsilon<double>()) const {
        const double larger_size = std::max(std::abs(this->si_size), std::abs(other.si_size));
        return this->quantity == other.quantity && std::abs(this->si_size - other.si_size) <= relative_tolerance * larger_size;
    }
};

namespace units {

inline constexpr Unit metre{Quantity::Length, 1.0};
inline constexpr Unit radian{Quantity::Angle, 1.0};
inline constexpr Unit degree{Quantity::Angle, std::numbers::pi / 180};

}

// `value` in `from`, expressed in `to`; both must measure the same quantity.
[[nodiscard]]
constexpr double convert(const double value, const Unit &from, const Unit &to) {
    ASSERT(from.quantity == to.quantity);
    return value * from.si_size / to.si_size;
}

}
