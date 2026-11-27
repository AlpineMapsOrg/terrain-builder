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

#include <libassert/assert.hpp>

namespace gis::srs {

// The kind of a system's coordinates; a compound system has the kind of its horizontal part.
enum class Kind {
    // Latitude and longitude on an ellipsoid.
    Geographic,
    // Coordinates on a map projection of a geographic system.
    Projected,
    // Cartesian coordinates with their origin at the earth's centre.
    Geocentric,
    // Heights or depths only.
    Vertical,
    // Coordinates relative to a local origin that is not tied to the earth's shape, such as a building's.
    Engineering,
    // Any other kind, such as temporal or parametric.
    Other,
};

// Whether systems of the kind are based on a geodetic datum.
[[nodiscard]]
inline bool is_geodetic(const Kind kind) {
    switch (kind) {
    case Kind::Geographic:
    case Kind::Projected:
    case Kind::Geocentric:
        return true;
    case Kind::Vertical:
    case Kind::Engineering:
    case Kind::Other:
        return false;
    }
    UNREACHABLE("invalid kind", kind);
}

}
