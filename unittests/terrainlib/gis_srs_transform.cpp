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

#include <optional>
#include <vector>

#include <glm/glm.hpp>

#include "../result_helpers.h"
#include "error/format.h"
#include "gis/srs/System.h"
#include "gis/srs/Transform.h"

TEST_CASE("get_transform fails between spatial references of different celestial bodies", "[gis][srs][Transform]") {
    const gis::srs::System earth = REQUIRE_VALUE(gis::srs::from_user_input("EPSG:4326"));
    const gis::srs::System mars = REQUIRE_VALUE(gis::srs::from_user_input("IAU_2015:49900"));

    const auto error = REQUIRE_ERROR(gis::srs::get_transform(earth, mars));
    CHECK(error.holds<gis::srs::UnsupportedTransform>());
}

// The orthographic projection onto the north pole maps no point south of the equator.
TEST_CASE("apply fails for points outside the projection domain", "[gis][srs][Transform]") {
    const gis::srs::System geographic = REQUIRE_VALUE(gis::srs::from_user_input("EPSG:4326"));
    const gis::srs::System orthographic = REQUIRE_VALUE(gis::srs::from_user_input("ESRI:102035"));
    const gis::srs::Transform transform = REQUIRE_VALUE(gis::srs::get_transform(geographic, orthographic));

    std::optional<alp::Error<gis::UntransformablePoint>> error_opt;
    SECTION("apply in 2d") {
        error_opt = REQUIRE_ERROR(transform.apply(glm::dvec2(10, -20)));
    }
    SECTION("apply in 3d") {
        error_opt = REQUIRE_ERROR(transform.apply(glm::dvec3(10, -20, 300)));
    }
    SECTION("apply_inplace in 2d") {
        std::vector<glm::dvec2> points = {{10, 20}, {30, -40}, {50, 60}};
        error_opt = REQUIRE_ERROR(transform.apply_inplace(&points));
    }
    SECTION("apply_inplace in 3d") {
        std::vector<glm::dvec3> points = {{10, 20, 300}, {30, -40, 500}, {50, 60, 700}};
        error_opt = REQUIRE_ERROR(transform.apply_inplace(&points));
    }
    SECTION("apply_into from 3d to 2d") {
        const std::vector<glm::dvec3> points = {{10, 20, 300}, {30, -40, 500}, {50, 60, 700}};
        std::vector<glm::dvec2> out(points.size());
        error_opt = REQUIRE_ERROR(transform.apply_into(points, &out));
    }

    CHECK(error_opt.value().holds<gis::UntransformablePoint>());
}

TEST_CASE("apply succeeds on an empty range", "[gis][srs][Transform]") {
    const gis::srs::System geographic = REQUIRE_VALUE(gis::srs::from_user_input("EPSG:4326"));
    const gis::srs::System mercator = REQUIRE_VALUE(gis::srs::from_user_input("EPSG:3857"));
    const gis::srs::Transform transform = REQUIRE_VALUE(gis::srs::get_transform(geographic, mercator));

    SECTION("apply_inplace") {
        std::vector<glm::dvec2> points;
        REQUIRE_VALUE(transform.apply_inplace(&points));
        CHECK(points.empty());
    }
    SECTION("apply_into") {
        const std::vector<glm::dvec3> points;
        std::vector<glm::dvec2> out;
        REQUIRE_VALUE(transform.apply_into(points, &out));
        CHECK(out.empty());
    }
}
