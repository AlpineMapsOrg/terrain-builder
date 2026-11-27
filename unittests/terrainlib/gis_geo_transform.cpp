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
#include <limits>

#include <glm/glm.hpp>

#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "gis/GeoTransform.h"

TEST_CASE("apply maps a pixel into the srs", "[gis][GeoTransform]") {
    const gis::GeoTransform transform = REQUIRE_VALUE(gis::GeoTransform::from_coefficients({10, 3, 1, 50, 2, -2}));

    CHECK(transform.apply(glm::dvec2(0, 0)) == glm::dvec2(10, 50));
    CHECK(transform.apply(glm::dvec2(3, 1)) == glm::dvec2(20, 54));
}

TEST_CASE("apply_inverse maps a point in the srs to its pixel", "[gis][GeoTransform]") {
    const gis::GeoTransform transform = REQUIRE_VALUE(gis::GeoTransform::from_coefficients({10, 3, 1, 50, 2, -2}));

    CHECK(transform.apply_inverse(glm::dvec2(10, 50)) == glm::dvec2(0, 0));
    CHECK(transform.apply_inverse(glm::dvec2(20, 54)) == glm::dvec2(3, 1));
}

TEST_CASE("is_axis_aligned is false when either rotation term is set", "[gis][GeoTransform]") {
    SECTION("no rotation") {
        const gis::GeoTransform transform = REQUIRE_VALUE(gis::GeoTransform::from_coefficients({10, 2, 0, 50, 0, -4}));
        CHECK(transform.is_axis_aligned());
    }
    SECTION("row rotation") {
        const gis::GeoTransform transform = REQUIRE_VALUE(gis::GeoTransform::from_coefficients({10, 2, 1, 50, 0, -4}));
        CHECK(!transform.is_axis_aligned());
    }
    SECTION("column rotation") {
        const gis::GeoTransform transform = REQUIRE_VALUE(gis::GeoTransform::from_coefficients({10, 2, 0, 50, 1, -4}));
        CHECK(!transform.is_axis_aligned());
    }
}

TEST_CASE("pixel_size is the extent of one pixel along x and y", "[gis][GeoTransform]") {
    const gis::GeoTransform transform = REQUIRE_VALUE(gis::GeoTransform::from_coefficients({10, 3, 1, 50, 2, -2}));

    CHECK(transform.pixel_size() == glm::dvec2(3, -2));
}

TEST_CASE("from_coefficients fails for a coefficient that is not finite", "[gis][GeoTransform]") {
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    const std::array<double, 6> coefficients = GENERATE_COPY(
        std::array<double, 6>{nan, 3, 1, 50, 2, -2},
        std::array<double, 6>{10, nan, 1, 50, 2, -2},
        std::array<double, 6>{10, 3, nan, 50, 2, -2},
        std::array<double, 6>{10, 3, 1, nan, 2, -2},
        std::array<double, 6>{10, 3, 1, 50, nan, -2},
        std::array<double, 6>{10, 3, 1, 50, 2, nan});

    const auto error = REQUIRE_ERROR(gis::GeoTransform::from_coefficients(coefficients));

    CHECK(error.holds_code(gis::InvalidGeoTransform::NotFinite));
}

TEST_CASE("from_coefficients fails for coefficients that cannot be inverted", "[gis][GeoTransform]") {
    const std::array<double, 6> coefficients = GENERATE(
        std::array<double, 6>{10, 0, 0, 50, 0, 0},
        std::array<double, 6>{10, 1, 2, 50, 2, 4});

    const auto error = REQUIRE_ERROR(gis::GeoTransform::from_coefficients(coefficients));

    CHECK(error.holds_code(gis::InvalidGeoTransform::NotInvertible));
}
