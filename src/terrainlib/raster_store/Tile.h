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

#pragma once

#include <cstdint>

#include <radix/raster.h>

namespace raster_store {

inline constexpr uint32_t default_tile_side = 4096;

template <typename PixelType>
struct Tile {
    explicit Tile(const uint32_t side = default_tile_side)
        : data(side)
        , attribution(glm::uvec2(side), uint16_t { 0 })
    {
    }

    radix::Raster<PixelType> data;
    radix::Raster<uint16_t> attribution;
};

} // namespace raster_store
