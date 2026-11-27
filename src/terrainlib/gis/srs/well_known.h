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

#include "error/unwrap.h"
#include "gis/srs/System.h"

namespace gis::srs {

// WGS84's geocentric system, EPSG:4978.
[[nodiscard]]
inline System ecef() {
    return alp::unwrap(from_epsg(4978));
}

// Web Mercator, EPSG:3857.
[[nodiscard]]
inline System webmercator() {
    return alp::unwrap(from_epsg(3857));
}

// WGS84's geographic system, EPSG:4326.
[[nodiscard]]
inline System wgs84() {
    return alp::unwrap(from_epsg(4326));
}

// MGI's geographic system, EPSG:4312.
[[nodiscard]]
inline System mgi() {
    return alp::unwrap(from_epsg(4312));
}

}
