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

#include <catch2/catch_test_macros.hpp>
#include <glm/glm.hpp>
#include <radix/geometry.h>
#include <radix/tile.h>

#include "../result_helpers.h"
#include "gis/srs/System.h"
#include "gis/srs/Transform.h"
#include "gis/srs/coverage.h"
#include "gis/srs/well_known.h"
#include "native_regions.h"

namespace {

// ECEF bounds of the given longitude/latitude bounds in degrees, between heights 0 and 200 m.
radix::geometry::Aabb3d ecef_bounds(const glm::dvec2& longitudes, const glm::dvec2& latitudes)
{
    return REQUIRE_VALUE(gis::srs::coverage::compute_ecef(gis::srs::wgs84(), { { longitudes.x, latitudes.x, 0 }, { longitudes.y, latitudes.y, 200 } }));
}

} // namespace

TEST_CASE("native read windows follow the longitudes of the dataset", "[native_regions]")
{
    const auto wgs84 = gis::srs::wgs84();
    const auto near_minus_175 = ecef_bounds({ -175.1, -174.9 }, { -0.1, 0.1 });

    SECTION("dataset stored at [0, 360]")
    {
        const radix::tile::SrsBounds native_bounds { { 0, -10 }, { 360, 10 } };
        const auto regions = REQUIRE_VALUE(sf::compute_native_regions(wgs84, native_bounds, near_minus_175));
        const auto windows = sf::clip_to_native_bounds(regions, native_bounds);
        REQUIRE(windows.size() == 1);
        CHECK(windows[0].min.x > 184.8);
        CHECK(windows[0].max.x < 185.2);
    }

    SECTION("dataset stored at [180, 190]")
    {
        const radix::tile::SrsBounds native_bounds { { 180, -5 }, { 190, 5 } };
        const auto regions = REQUIRE_VALUE(sf::compute_native_regions(wgs84, native_bounds, near_minus_175));
        const auto windows = sf::clip_to_native_bounds(regions, native_bounds);
        REQUIRE(windows.size() == 1);
        CHECK(windows[0].min.x > 184.8);
        CHECK(windows[0].max.x < 185.2);
        const auto near_175_regions = REQUIRE_VALUE(sf::compute_native_regions(wgs84, native_bounds, ecef_bounds({ 174.9, 175.1 }, { -0.1, 0.1 })));
        CHECK(sf::clip_to_native_bounds(near_175_regions, native_bounds).empty());
    }

    SECTION("bounds across the antimeridian are read on both edges of the dataset")
    {
        const radix::tile::SrsBounds native_bounds { { -180, -10 }, { 180, 10 } };
        const auto regions = REQUIRE_VALUE(sf::compute_native_regions(wgs84, native_bounds, ecef_bounds({ 179.9, 180.1 }, { -0.1, 0.1 })));
        const auto windows = sf::clip_to_native_bounds(regions, native_bounds);
        REQUIRE(windows.size() == 2);
        const auto& east = windows[0].max.x == 180 ? windows[0] : windows[1];
        const auto& west = windows[0].max.x == 180 ? windows[1] : windows[0];
        CHECK(east.max.x == 180);
        CHECK(east.min.x > 179.8);
        CHECK(west.min.x == -180);
        CHECK(west.max.x < -179.8);
    }

    SECTION("projected datasets are not shifted")
    {
        const auto austria_lambert = REQUIRE_VALUE(gis::srs::from_epsg(31287));
        const radix::tile::SrsBounds native_bounds { { 100000, 250000 }, { 700000, 600000 } };
        const glm::dvec3 vienna = REQUIRE_VALUE(gis::srs::transform_point(wgs84, austria_lambert, glm::dvec3(16.37, 48.2, 0)));
        const auto regions = REQUIRE_VALUE(sf::compute_native_regions(austria_lambert, native_bounds, ecef_bounds({ 16.36, 16.38 }, { 48.19, 48.21 })));
        const auto windows = sf::clip_to_native_bounds(regions, native_bounds);
        REQUIRE(windows.size() == 1);
        CHECK(windows[0].contains(glm::dvec2(vienna)));
        CHECK(windows[0].width() < 3000);
    }
}

TEST_CASE("intersects_native_bounds tells whether bounds reach into the native bounds", "[native_regions]")
{
    const auto wgs84 = gis::srs::wgs84();
    const radix::tile::SrsBounds native_bounds { { 180, -5 }, { 190, 5 } };

    SECTION("inside")
    {
        CHECK(REQUIRE_VALUE(sf::intersects_native_bounds(wgs84, native_bounds, ecef_bounds({ -175.1, -174.9 }, { -0.1, 0.1 }))));
    }

    SECTION("outside")
    {
        CHECK_FALSE(REQUIRE_VALUE(sf::intersects_native_bounds(wgs84, native_bounds, ecef_bounds({ 174.9, 175.1 }, { -0.1, 0.1 }))));
    }

    SECTION("unreachable by the dataset's srs")
    {
        // The bounds lie 90 degrees east of the central meridian of UTM zone 33N, at 15 degrees, which the zone cannot express.
        const auto utm_33n = REQUIRE_VALUE(gis::srs::from_epsg(32633));
        const radix::tile::SrsBounds utm_bounds { { 400000, 5100000 }, { 600000, 5300000 } };
        const radix::geometry::Aabb3d quarter_turn_east = ecef_bounds({ 104.9, 105.1 }, { -0.1, 0.1 });
        const auto regions_error = REQUIRE_ERROR(sf::compute_native_regions(utm_33n, utm_bounds, quarter_turn_east));
        REQUIRE(regions_error.holds<gis::UntransformablePoint>());
        CHECK_FALSE(REQUIRE_VALUE(sf::intersects_native_bounds(utm_33n, utm_bounds, quarter_turn_east)));
    }
}
