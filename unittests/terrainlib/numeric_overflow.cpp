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

#include "numeric/overflow.h"

TEST_CASE("mul_overflow writes a product that fits", "[numeric][overflow]") {
    SECTION("unsigned") {
        uint8_t result = 0;
        CHECK_FALSE(mul_overflow(uint8_t{15}, uint8_t{17}, result));
        CHECK(result == 255);
        CHECK_FALSE(mul_overflow(uint8_t{0}, uint8_t{255}, result));
        CHECK(result == 0);
    }
    SECTION("signed") {
        int8_t result = 0;
        CHECK_FALSE(mul_overflow(int8_t{-8}, int8_t{16}, result));
        CHECK(result == -128);
        CHECK_FALSE(mul_overflow(int8_t{-11}, int8_t{-11}, result));
        CHECK(result == 121);
    }
}

TEST_CASE("mul_overflow reports a product that does not fit and leaves the result unchanged", "[numeric][overflow]") {
    SECTION("unsigned") {
        uint8_t result = 7;
        CHECK(mul_overflow(uint8_t{16}, uint8_t{16}, result));
        CHECK(result == 7);
    }
    SECTION("signed") {
        int8_t result = 7;
        CHECK(mul_overflow(int8_t{-8}, int8_t{17}, result));
        CHECK(mul_overflow(int8_t{-1}, int8_t{-128}, result));
        CHECK(mul_overflow(int8_t{16}, int8_t{8}, result));
        CHECK(result == 7);
    }
}
