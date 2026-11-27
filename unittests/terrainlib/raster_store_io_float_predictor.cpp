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

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>

#include <catch2/catch_test_macros.hpp>

#include "raster_store/io/float_predictor.h"

TEST_CASE("float_predictor encode and decode match libtiff's Predictor 3 including plane and row boundaries", "[raster_store][io][float_predictor]")
{
    // Two rows of three Float32 values, so that both a plane boundary and a row boundary fall inside the data.
    const std::array<uint32_t, 6> bits = { 0x3f800000, 0x40000000, 0xc0400000, 0x80000000, 0x7fc01234, 0x00000001 };
    // Raw Deflate strip decompressed from libtiff 4.7.0 with Predictor=3, SampleFormat=IEEEFP, BitsPerSample=32, SamplesPerPixel=1 and width 3.
    const std::array<uint8_t, 24> reference = {
        0x3f, 0x01, 0x80, 0xc0, 0x80, 0x40, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x81, 0x00, 0xc0, 0x40, 0x00, 0x12, 0xee, 0x00, 0x34, 0xcd
    };

    std::array<uint32_t, 6> encoded = bits;
    raster_store::io::float_predictor::encode(std::as_writable_bytes(std::span(encoded)), 3);
    CHECK(std::ranges::equal(std::as_bytes(std::span(encoded)), std::as_bytes(std::span(reference))));

    // Decode the independent reference, even if encoding fails the comparison.
    std::array<uint8_t, 24> decoded = reference;
    raster_store::io::float_predictor::decode(std::as_writable_bytes(std::span(decoded)), 3);
    CHECK(std::ranges::equal(std::as_bytes(std::span(decoded)), std::as_bytes(std::span(bits))));
}
