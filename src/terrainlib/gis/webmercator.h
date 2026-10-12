/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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

#include <radix/tile.h>

namespace gis::webmercator {

/// Half the side length of the square Web Mercator world in metres.
constexpr double half_extent = 20037508.342789244;
/// Latitude in degrees at which Web Mercator y reaches half_extent.
constexpr double latitude_limit = 85.0511287798066;
/// Web Mercator bounds of an XYZ tile; valid up to zoom level 32.
radix::tile::SrsBounds tile_bounds(const radix::tile::Id& key);

} // namespace gis::webmercator
