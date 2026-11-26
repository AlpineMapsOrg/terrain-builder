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

#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "gis/srs/System.h"
#include "gis/srs/geographic.h"
#include "gis/srs/well_known.h"

#include <catch2/catch_test_macros.hpp>
#include <ogr_spatialref.h>

#include <utility>
#include <vector>

TEST_CASE("wrap_longitudes shifts bounds into the half circle around the longitude of the centre", "[gis][srs][geographic]")
{
    const auto wrapped = REQUIRE_VALUE(gis::srs::wrap_longitudes(gis::srs::wgs84(), { { -170, -5 }, { -160, 5 } }, { 185, 0 }));

    CHECK_THAT(wrapped, UnorderedWithinEpsilon(std::vector<radix::tile::SrsBounds> { { { 190, -5 }, { 200, 5 } } }));
}

TEST_CASE("wrap_longitudes wraps the longitude wherever it is among the data axes", "[gis][srs][geographic]")
{
    OGRSpatialReference authority_srs;
    REQUIRE(authority_srs.importFromEPSG(4326) == OGRERR_NONE);
    const gis::srs::System latitude_first(std::move(authority_srs));

    const auto wrapped = REQUIRE_VALUE(gis::srs::wrap_longitudes(latitude_first, { { -5, -170 }, { 5, -160 } }, { 0, 185 }));

    CHECK_THAT(wrapped, UnorderedWithinEpsilon(std::vector<radix::tile::SrsBounds> { { { -5, 190 }, { 5, 200 } } }));
}

TEST_CASE("wrap_longitudes wraps by a full circle in the unit of the longitude", "[gis][srs][geographic]")
{
    const auto grads = REQUIRE_VALUE(gis::srs::from_user_input(
        R"(GEOGCRS["grads",DATUM["World Geodetic System 1984",ELLIPSOID["WGS 84",6378137,298.257223563]],CS[ellipsoidal,2],AXIS["longitude",east,ANGLEUNIT["grad",0.015707963267949]],AXIS["latitude",north,ANGLEUNIT["grad",0.015707963267949]]])"));

    const auto wrapped = REQUIRE_VALUE(gis::srs::wrap_longitudes(grads, { { -210, 10 }, { -200, 20 } }, { 200, 0 }));

    CHECK_THAT(wrapped, UnorderedWithinEpsilon(std::vector<radix::tile::SrsBounds> { { { 190, 10 }, { 200, 20 } } }));
}

TEST_CASE("wrap_longitudes fails for a system that is not geographic", "[gis][srs][geographic]")
{
    const auto error = REQUIRE_ERROR(gis::srs::wrap_longitudes(gis::srs::webmercator(), { { -170, -5 }, { -160, 5 } }, { 185, 0 }));

    CHECK(error.holds<gis::srs::NotGeographic>());
}

// TODO: enable once gis::srs::System reads per-axis units; System::axes() reports the metre axes of a geographic system as radians (see the merge todo).
/*
TEST_CASE("wrap_longitudes fails without an angular longitude", "[gis][srs][geographic]")
{
    const auto metres = REQUIRE_VALUE(gis::srs::from_user_input(
        R"(GEOGCRS["metres",DATUM["World Geodetic System 1984",ELLIPSOID["WGS 84",6378137,298.257223563]],CS[ellipsoidal,2],AXIS["longitude",east,LENGTHUNIT["metre",1]],AXIS["latitude",north,LENGTHUNIT["metre",1]]])"));

    const auto error = REQUIRE_ERROR(gis::srs::wrap_longitudes(metres, { { -170, -5 }, { -160, 5 } }, { 185, 0 }));

    CHECK(error.holds<gis::srs::UnsupportedAxisOrder>());
}
*/
