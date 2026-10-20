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

#include <cstddef>
#include <span>

namespace raster_store::io::float_predictor {

// TIFF Predictor 3 for single-channel Float32 rows in native byte order.
// width must be positive and data must contain complete rows of width * 4 bytes.
// The encoded byte planes are most-significant first; differencing continues
// across plane boundaries, resetting only at the beginning of each row.
void encode(std::span<std::byte> data, std::size_t width);
void decode(std::span<std::byte> data, std::size_t width);

} // namespace raster_store::io::float_predictor
