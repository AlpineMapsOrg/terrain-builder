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
#include <ranges>
#include <string>
#include <vector>

#include "../result_helpers.h"
#include "error/format.h"
#include "gis/OgrError.h"
#include "gis/srs/Axis.h"
#include "gis/srs/System.h"

TEST_CASE("from_epsg fails for an unknown code", "[gis][srs][System]") {
    CHECK(REQUIRE_ERROR(gis::srs::from_epsg(999999)).holds<gis::OgrError>());
}

TEST_CASE("from_epsg gives a system with its data axes in traditional order", "[gis][srs][System]") {
    const gis::srs::System geographic = REQUIRE_VALUE(gis::srs::from_epsg(4326));

    CHECK(geographic.as_gdal().GetDataAxisToSRSAxisMapping() == std::vector<int>{2, 1});
}

TEST_CASE("from_user_input fails for invalid input", "[gis][srs][System]") {
    CHECK(REQUIRE_ERROR(gis::srs::from_user_input("not a spatial reference")).holds<gis::OgrError>());
}

TEST_CASE("from_user_input gives a system with its data axes in traditional order", "[gis][srs][System]") {
    const gis::srs::System geographic = REQUIRE_VALUE(gis::srs::from_user_input("EPSG:4326"));

    CHECK(geographic.as_gdal().GetDataAxisToSRSAxisMapping() == std::vector<int>{2, 1});
}

TEST_CASE("kind gives the kind of the coordinates, which for a compound system is that of its horizontal part", "[gis][srs][System]") {
    std::string input;
    gis::srs::Kind expected;
    SECTION("geographic") {
        input = "EPSG:4326";
        expected = gis::srs::Kind::Geographic;
    }
    SECTION("projected") {
        input = "EPSG:3857";
        expected = gis::srs::Kind::Projected;
    }
    SECTION("geocentric") {
        input = "EPSG:4978";
        expected = gis::srs::Kind::Geocentric;
    }
    SECTION("compound with a geographic horizontal part") {
        input = "EPSG:4326+3855";
        expected = gis::srs::Kind::Geographic;
    }
    SECTION("compound with a projected horizontal part") {
        input = "EPSG:31287+5778";
        expected = gis::srs::Kind::Projected;
    }
    const gis::srs::System system = REQUIRE_VALUE(gis::srs::from_user_input(input));

    CHECK(system.kind() == expected);
}

TEST_CASE("axes gives the axes in data order", "[gis][srs][System]") {
    SECTION("geographic, whose authority order is latitude first") {
        const gis::srs::System system = REQUIRE_VALUE(gis::srs::from_epsg(4326));
        const auto directions = system.axes() | std::views::transform(&gis::srs::Axis::direction) | std::ranges::to<std::vector>();
        CHECK(directions == std::vector{gis::srs::AxisDirection::East, gis::srs::AxisDirection::North});
    }
    SECTION("projected, whose authority order is northing first") {
        const gis::srs::System system = REQUIRE_VALUE(gis::srs::from_epsg(31287));
        const auto directions = system.axes() | std::views::transform(&gis::srs::Axis::direction) | std::ranges::to<std::vector>();
        CHECK(directions == std::vector{gis::srs::AxisDirection::East, gis::srs::AxisDirection::North});
    }
    SECTION("compound") {
        const gis::srs::System system = REQUIRE_VALUE(gis::srs::from_user_input("EPSG:31287+5778"));
        const auto directions = system.axes() | std::views::transform(&gis::srs::Axis::direction) | std::ranges::to<std::vector>();
        CHECK(directions == std::vector{gis::srs::AxisDirection::East, gis::srs::AxisDirection::North, gis::srs::AxisDirection::Up});
    }
}

TEST_CASE("axes gives angles for geographic horizontal axes and lengths for the others", "[gis][srs][System]") {
    SECTION("geographic with a height") {
        const gis::srs::System system = REQUIRE_VALUE(gis::srs::from_epsg(4979));
        const auto quantities = system.axes() | std::views::transform(&gis::srs::Axis::unit) | std::views::transform(&gis::srs::Unit::quantity) | std::ranges::to<std::vector>();
        CHECK(quantities == std::vector{gis::srs::Quantity::Angle, gis::srs::Quantity::Angle, gis::srs::Quantity::Length});
    }
    SECTION("compound") {
        const gis::srs::System system = REQUIRE_VALUE(gis::srs::from_user_input("EPSG:31287+5778"));
        const auto quantities = system.axes() | std::views::transform(&gis::srs::Axis::unit) | std::views::transform(&gis::srs::Unit::quantity) | std::ranges::to<std::vector>();
        CHECK(quantities == std::vector{gis::srs::Quantity::Length, gis::srs::Quantity::Length, gis::srs::Quantity::Length});
    }
}

TEST_CASE("format_epsg_code formats the EPSG code of a system, or of both parts of a compound system", "[gis][srs][System]") {
    SECTION("single") {
        const gis::srs::System system = REQUIRE_VALUE(gis::srs::from_epsg(4326));
        CHECK(gis::srs::format_epsg_code(system) == "EPSG:4326");
    }
    SECTION("projected compound") {
        const gis::srs::System system = REQUIRE_VALUE(gis::srs::from_user_input("EPSG:31287+5778"));
        CHECK(gis::srs::format_epsg_code(system) == "EPSG:31287+5778");
    }
    SECTION("geographic compound") {
        const gis::srs::System system = REQUIRE_VALUE(gis::srs::from_user_input("EPSG:4326+5773"));
        CHECK(gis::srs::format_epsg_code(system) == "EPSG:4326+5773");
    }
    SECTION("without a code") {
        const gis::srs::System system = REQUIRE_VALUE(gis::srs::from_user_input(
            R"(GEOGCRS["sphere",DATUM["sphere",ELLIPSOID["sphere",6371000,0]],CS[ellipsoidal,2],AXIS["longitude",east,ANGLEUNIT["degree",0.0174532925199433]],AXIS["latitude",north,ANGLEUNIT["degree",0.0174532925199433]]])"));
        CHECK(gis::srs::format_epsg_code(system) == std::nullopt);
    }
}
