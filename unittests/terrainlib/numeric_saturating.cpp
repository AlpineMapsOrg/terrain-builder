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

#include "numeric/saturating.h"

TEST_CASE("saturating_mul returns the product if it fits, otherwise the bound it passes", "[numeric][saturating]") {
    SECTION("unsigned") {
        STATIC_CHECK(saturating_mul(uint8_t{15}, uint8_t{17}) == uint8_t{255});
        STATIC_CHECK(saturating_mul(uint8_t{16}, uint8_t{16}) == uint8_t{255});
    }
    SECTION("signed") {
        STATIC_CHECK(saturating_mul(int8_t{-11}, int8_t{-11}) == int8_t{121});
        STATIC_CHECK(saturating_mul(int8_t{16}, int8_t{8}) == int8_t{127});
        STATIC_CHECK(saturating_mul(int8_t{-8}, int8_t{-17}) == int8_t{127});
        STATIC_CHECK(saturating_mul(int8_t{-8}, int8_t{17}) == int8_t{-128});
        STATIC_CHECK(saturating_mul(int8_t{8}, int8_t{-17}) == int8_t{-128});
    }
}
