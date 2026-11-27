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

#include <numbers>

#include <catch2/catch_test_macros.hpp>

#include "../catch2_helpers.h"
#include "gis/srs/Unit.h"

TEST_CASE("Unit::is_approx compares sizes up to the relative tolerance", "[gis][srs][Unit]") {
    const gis::srs::Unit inch{gis::srs::Quantity::Length, 0.0254};
    const gis::srs::Unit us_survey_inch{gis::srs::Quantity::Length, 100.0 / 3937};

    SECTION("a tolerance below their difference tells them apart") {
        CHECK_FALSE(inch.is_approx(us_survey_inch, 1e-7));
    }
    SECTION("a tolerance above their difference does not") {
        CHECK(inch.is_approx(us_survey_inch, 1e-5));
    }
}

TEST_CASE("Unit::is_approx rejects another quantity of the same size", "[gis][srs][Unit]") {
    CHECK_FALSE(gis::srs::units::radian.is_approx(gis::srs::units::metre));
}

TEST_CASE("convert expresses a value in another unit of the same quantity", "[gis][srs][Unit]") {
    CHECK_THAT(gis::srs::convert(2 * std::numbers::pi, gis::srs::units::radian, gis::srs::units::degree), WithinEpsilon(360));
    CHECK_THAT(gis::srs::convert(180, gis::srs::units::degree, gis::srs::units::radian), WithinEpsilon(std::numbers::pi));
}
