/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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

#include "gis/webmercator.h"

#include <cstdint>

#include <glm/glm.hpp>

namespace gis::webmercator {

radix::tile::SrsBounds compute_tile_bounds(const radix::tile::Id& key) {
    const uint64_t tile_count = uint64_t{1} << key.zoom_level;
    const double tile_size = 2 * half_extent / tile_count;
    // XYZ counts rows from the north, while the bounds grow northwards.
    const glm::dvec2 grid_coords(key.coords.x, tile_count - 1 - key.coords.y);
    const glm::dvec2 tile_min = glm::dvec2(-half_extent) + grid_coords * tile_size;
    const glm::dvec2 tile_max = tile_min + tile_size;
    return { tile_min, tile_max };
}

} // namespace gis::webmercator
