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

#include <array>

#include <catch2/catch_test_macros.hpp>
#include <radix/geometry.h>
#include <zpp_bits.h>

#include "io/bytes.h"
#include "io/radix_serialization.h"

TEST_CASE("radix AABBs serialize their minimum then their maximum", "[io][radix_serialization]") {
    const radix::geometry::Aabb3d bounds { { 0.5, 1.5, 2.5 }, { 3.5, 4.5, 5.5 } };

    io::bytes::vector bytes;
    zpp::bits::out output(bytes);
    REQUIRE(!zpp::bits::failure(output(bounds)));
    io::bytes::vector expected_bytes;
    zpp::bits::out expected_output(expected_bytes);
    REQUIRE(!zpp::bits::failure(expected_output(std::array { 0.5, 1.5, 2.5, 3.5, 4.5, 5.5 })));
    CHECK(bytes == expected_bytes);

    radix::geometry::Aabb3d decoded {};
    zpp::bits::in input(bytes);
    REQUIRE(!zpp::bits::failure(input(decoded)));
    CHECK(decoded == bounds);
}
