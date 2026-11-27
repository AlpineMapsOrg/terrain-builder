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

#include "float_predictor.h"

#include <bit>
#include <vector>

#include <libassert/assert.hpp>

namespace raster_store::io::float_predictor {
namespace {
    constexpr std::size_t native_byte(const std::size_t significance)
    {
        static_assert(std::endian::native == std::endian::little || std::endian::native == std::endian::big);
        return std::endian::native == std::endian::little ? 3 - significance : significance;
    }
} // namespace

// Byte layout and differencing follow libtiff's fpDiff / fpAcc:
// https://github.com/libsdl-org/libtiff/blob/master/libtiff/tif_predict.c
void encode(const std::span<std::byte> data, const std::size_t width)
{
    ASSERT(width > 0 && width <= data.size() / 4 && data.size() % (width * 4) == 0);
    std::vector<std::byte> row(width * 4);
    for (std::size_t offset = 0; offset < data.size(); offset += row.size()) {
        const auto source = data.subspan(offset, row.size());
        for (std::size_t byte = 0; byte < 4; ++byte) {
            for (std::size_t x = 0; x < width; ++x) {
                row[byte * width + x] = source[4 * x + native_byte(byte)];
            }
        }
        source[0] = row[0];
        for (std::size_t i = 1; i < row.size(); ++i) {
            source[i] = std::byte((std::to_integer<unsigned>(row[i]) - std::to_integer<unsigned>(row[i - 1])) & 0xff);
        }
    }
}

void decode(const std::span<std::byte> data, const std::size_t width)
{
    ASSERT(width > 0 && width <= data.size() / 4 && data.size() % (width * 4) == 0);
    std::vector<std::byte> row(width * 4);
    for (std::size_t offset = 0; offset < data.size(); offset += row.size()) {
        const auto destination = data.subspan(offset, row.size());
        row[0] = destination[0];
        for (std::size_t i = 1; i < row.size(); ++i) {
            row[i] = std::byte((std::to_integer<unsigned>(destination[i]) + std::to_integer<unsigned>(row[i - 1])) & 0xff);
        }
        for (std::size_t byte = 0; byte < 4; ++byte) {
            for (std::size_t x = 0; x < width; ++x) {
                destination[4 * x + native_byte(byte)] = row[byte * width + x];
            }
        }
    }
}

} // namespace raster_store::io::float_predictor
