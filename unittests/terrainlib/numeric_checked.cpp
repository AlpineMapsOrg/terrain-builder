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

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <optional>

#include "numeric/checked.h"

TEST_CASE("checked_add returns the sum if it fits, otherwise nothing", "[numeric][checked]") {
    STATIC_CHECK(checked_add(uint8_t{200}, uint8_t{55}) == uint8_t{255});
    STATIC_CHECK(checked_add(uint8_t{200}, uint8_t{56}) == std::nullopt);
}

TEST_CASE("checked_sub returns the difference if it fits, otherwise nothing", "[numeric][checked]") {
    STATIC_CHECK(checked_sub(uint8_t{5}, uint8_t{5}) == uint8_t{0});
    STATIC_CHECK(checked_sub(uint8_t{5}, uint8_t{6}) == std::nullopt);
}

TEST_CASE("checked_mul returns the product if it fits, otherwise nothing", "[numeric][checked]") {
    SECTION("unsigned") {
        STATIC_CHECK(checked_mul(uint8_t{15}, uint8_t{17}) == uint8_t{255});
        STATIC_CHECK(checked_mul(uint8_t{16}, uint8_t{16}) == std::nullopt);
    }
    SECTION("signed") {
        STATIC_CHECK(checked_mul(int8_t{-8}, int8_t{16}) == int8_t{-128});
        STATIC_CHECK(checked_mul(int8_t{-8}, int8_t{17}) == std::nullopt);
        STATIC_CHECK(checked_mul(int8_t{-1}, int8_t{-128}) == std::nullopt);
    }
}
