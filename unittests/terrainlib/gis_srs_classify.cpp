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
#include <utility>

#include <ogr_spatialref.h>

#include "../result_helpers.h"
#include "gis/srs/Axis.h"
#include "gis/srs/System.h"
#include "gis/srs/Unit.h"
#include "gis/srs/classify.h"
#include "gis/srs/well_known.h"

using gis::srs::AxisDirection;

TEST_CASE("has_axis_order matches the leading data axes in order", "[gis][srs][classify]") {
    SECTION("data order") {
        const gis::srs::System wgs84 = gis::srs::wgs84();

        CHECK(gis::srs::has_axis_order(wgs84, {AxisDirection::East, AxisDirection::North}));
        CHECK_FALSE(gis::srs::has_axis_order(wgs84, {AxisDirection::North, AxisDirection::East}));
    }
    SECTION("a prefix matches") {
        CHECK(gis::srs::has_axis_order(gis::srs::wgs84(), {AxisDirection::East}));
    }
    SECTION("more directions than axes") {
        CHECK_FALSE(gis::srs::has_axis_order(gis::srs::wgs84(), {AxisDirection::East, AxisDirection::North, AxisDirection::Up}));
    }
}

TEST_CASE("is_east_north_axis_order is true when the data axes start with the easting or longitude, then the northing or latitude", "[gis][srs][classify]") {
    SECTION("east, then north") {
        OGRSpatialReference authority_srs;
        REQUIRE(authority_srs.importFromEPSG(4326) == OGRERR_NONE);
        const gis::srs::System latitude_first(std::move(authority_srs));

        CHECK(gis::srs::is_east_north_axis_order(gis::srs::wgs84()));
        CHECK(gis::srs::is_east_north_axis_order(gis::srs::webmercator()));
        CHECK_FALSE(gis::srs::is_east_north_axis_order(latitude_first));
    }
    SECTION("polar") {
        // Both axes of these systems point north, or both south, along meridians.
        OGRSpatialReference authority_srs;
        REQUIRE(authority_srs.importFromEPSG(32661) == OGRERR_NONE);
        const gis::srs::System northing_first(std::move(authority_srs));

        CHECK(gis::srs::is_east_north_axis_order(REQUIRE_VALUE(gis::srs::from_epsg(3031))));
        CHECK(gis::srs::is_east_north_axis_order(REQUIRE_VALUE(gis::srs::from_epsg(3413))));
        CHECK(gis::srs::is_east_north_axis_order(REQUIRE_VALUE(gis::srs::from_epsg(32661))));
        CHECK_FALSE(gis::srs::is_east_north_axis_order(northing_first));
    }
}

TEST_CASE("find_axis_index gives the data index of the first axis pointing in a direction", "[gis][srs][classify]") {
    SECTION("data index") {
        OGRSpatialReference authority_srs;
        REQUIRE(authority_srs.importFromEPSG(4326) == OGRERR_NONE);
        const gis::srs::System latitude_first(std::move(authority_srs));
        const gis::srs::System geographic_3d = REQUIRE_VALUE(gis::srs::from_epsg(4979));

        CHECK(gis::srs::find_axis_index(gis::srs::wgs84(), AxisDirection::East) == 0);
        CHECK(gis::srs::find_axis_index(latitude_first, AxisDirection::East) == 1);
        CHECK(gis::srs::find_axis_index(geographic_3d, AxisDirection::Up) == 2);
    }
    SECTION("no matching axis") {
        CHECK(gis::srs::find_axis_index(gis::srs::ecef(), AxisDirection::East) == std::nullopt);
    }
}

TEST_CASE("longitude_latitude_axes gives the longitude and latitude in data order", "[gis][srs][classify]") {
    SECTION("longitude first") {
        const std::optional<gis::srs::LongitudeLatitudeAxes> axes_opt = gis::srs::longitude_latitude_axes(gis::srs::wgs84());

        REQUIRE(axes_opt.has_value());
        CHECK(axes_opt->longitude.index == 0);
        CHECK(axes_opt->longitude.axis.direction == AxisDirection::East);
        CHECK(axes_opt->latitude.index == 1);
        CHECK(axes_opt->latitude.axis.direction == AxisDirection::North);
    }
    SECTION("latitude first") {
        OGRSpatialReference authority_srs;
        REQUIRE(authority_srs.importFromEPSG(4326) == OGRERR_NONE);
        const gis::srs::System latitude_first(std::move(authority_srs));

        const std::optional<gis::srs::LongitudeLatitudeAxes> axes_opt = gis::srs::longitude_latitude_axes(latitude_first);

        REQUIRE(axes_opt.has_value());
        CHECK(axes_opt->longitude.index == 1);
        CHECK(axes_opt->longitude.axis.direction == AxisDirection::East);
        CHECK(axes_opt->latitude.index == 0);
        CHECK(axes_opt->latitude.axis.direction == AxisDirection::North);
    }
}

TEST_CASE("longitude_latitude_axes finds longitudes that grow westwards", "[gis][srs][classify]") {
    const gis::srs::System west_positive = REQUIRE_VALUE(gis::srs::from_user_input(
        R"(GEOGCRS["west positive",DATUM["World Geodetic System 1984",ELLIPSOID["WGS 84",6378137,298.257223563]],CS[ellipsoidal,2],AXIS["longitude",west,ANGLEUNIT["degree",0.0174532925199433]],AXIS["latitude",north,ANGLEUNIT["degree",0.0174532925199433]]])"));

    const std::optional<gis::srs::LongitudeLatitudeAxes> axes_opt = gis::srs::longitude_latitude_axes(west_positive);

    REQUIRE(axes_opt.has_value());
    CHECK(axes_opt->longitude.index == 0);
    CHECK(axes_opt->longitude.axis.direction == AxisDirection::West);
}

TEST_CASE("longitude_latitude_axes finds none in a system that is not geographic", "[gis][srs][classify]") {
    CHECK_FALSE(gis::srs::longitude_latitude_axes(gis::srs::webmercator()).has_value());
}

// TODO: enable once gis::srs::System reads per-axis units; System::axes() reports the metre axes of a geographic system as radians (see the merge todo).
/*
TEST_CASE("longitude_latitude_axes finds none on axes that are not angles", "[gis][srs][classify]") {
    const gis::srs::System metres = REQUIRE_VALUE(gis::srs::from_user_input(
        R"(GEOGCRS["metres",DATUM["World Geodetic System 1984",ELLIPSOID["WGS 84",6378137,298.257223563]],CS[ellipsoidal,2],AXIS["longitude",east,LENGTHUNIT["metre",1]],AXIS["latitude",north,LENGTHUNIT["metre",1]]])"));

    CHECK_FALSE(gis::srs::longitude_latitude_axes(metres).has_value());
}
*/

TEST_CASE("horizontal_unit gives the unit the horizontal data axes share", "[gis][srs][classify]") {
    SECTION("shared unit") {
        const std::optional<gis::srs::Unit> geographic_opt = gis::srs::horizontal_unit(gis::srs::wgs84());
        const std::optional<gis::srs::Unit> projected_opt = gis::srs::horizontal_unit(gis::srs::webmercator());

        REQUIRE(geographic_opt.has_value());
        CHECK(geographic_opt->is_approx(gis::srs::units::degree));
        REQUIRE(projected_opt.has_value());
        CHECK(projected_opt->is_approx(gis::srs::units::metre));
    }
    SECTION("height excluded") {
        const std::optional<gis::srs::Unit> unit_opt = gis::srs::horizontal_unit(REQUIRE_VALUE(gis::srs::from_epsg(4979)));

        REQUIRE(unit_opt.has_value());
        CHECK(unit_opt->is_approx(gis::srs::units::degree));
    }
    SECTION("differing units") {
        const gis::srs::System mixed = REQUIRE_VALUE(gis::srs::from_user_input(
            R"(GEOGCRS["mixed",DATUM["World Geodetic System 1984",ELLIPSOID["WGS 84",6378137,298.257223563]],CS[ellipsoidal,2],AXIS["longitude",east,ANGLEUNIT["degree",0.0174532925199433]],AXIS["latitude",north,ANGLEUNIT["grad",0.015707963267949]]])"));

        CHECK(gis::srs::horizontal_unit(mixed) == std::nullopt);
    }
    SECTION("no horizontal axes") {
        CHECK(gis::srs::horizontal_unit(gis::srs::ecef()) == std::nullopt);
        CHECK(gis::srs::horizontal_unit(REQUIRE_VALUE(gis::srs::from_epsg(6360))) == std::nullopt);
    }
}

TEST_CASE("has_horizontal_axes_in requires horizontal axes in the unit", "[gis][srs][classify]") {
    SECTION("in the unit") {
        CHECK(gis::srs::has_horizontal_axes_in(gis::srs::wgs84(), gis::srs::units::degree));
    }
    SECTION("in another unit") {
        CHECK_FALSE(gis::srs::has_horizontal_axes_in(gis::srs::wgs84(), gis::srs::units::metre));
    }
    SECTION("no horizontal axes") {
        CHECK_FALSE(gis::srs::has_horizontal_axes_in(gis::srs::ecef(), gis::srs::units::metre));
    }
}

TEST_CASE("vertical_unit gives the unit of the vertical data axes", "[gis][srs][classify]") {
    SECTION("vertical axis") {
        const gis::srs::Unit us_survey_foot{gis::srs::Quantity::Length, 1200.0 / 3937};
        const std::optional<gis::srs::Unit> geographic_3d_opt = gis::srs::vertical_unit(REQUIRE_VALUE(gis::srs::from_epsg(4979)));
        const std::optional<gis::srs::Unit> compound_opt = gis::srs::vertical_unit(REQUIRE_VALUE(gis::srs::from_user_input("EPSG:4326+5773")));
        const std::optional<gis::srs::Unit> vertical_opt = gis::srs::vertical_unit(REQUIRE_VALUE(gis::srs::from_epsg(6360)));

        REQUIRE(geographic_3d_opt.has_value());
        CHECK(geographic_3d_opt->is_approx(gis::srs::units::metre));
        REQUIRE(compound_opt.has_value());
        CHECK(compound_opt->is_approx(gis::srs::units::metre));
        REQUIRE(vertical_opt.has_value());
        CHECK(vertical_opt->is_approx(us_survey_foot));
    }
    SECTION("no vertical axis") {
        CHECK(gis::srs::vertical_unit(gis::srs::wgs84()) == std::nullopt);
        CHECK(gis::srs::vertical_unit(gis::srs::ecef()) == std::nullopt);
    }
}

TEST_CASE("has_vertical_axes_in requires vertical axes in the unit", "[gis][srs][classify]") {
    SECTION("in the unit") {
        CHECK(gis::srs::has_vertical_axes_in(REQUIRE_VALUE(gis::srs::from_epsg(4979)), gis::srs::units::metre));
    }
    SECTION("in another unit") {
        CHECK_FALSE(gis::srs::has_vertical_axes_in(REQUIRE_VALUE(gis::srs::from_epsg(6360)), gis::srs::units::metre));
    }
    SECTION("no vertical axes") {
        CHECK_FALSE(gis::srs::has_vertical_axes_in(gis::srs::wgs84(), gis::srs::units::metre));
    }
}
