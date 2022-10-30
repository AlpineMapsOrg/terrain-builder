/*****************************************************************************
 * Alpine Terrain Builder
 * Copyright (C) 2022 alpinemaps.org
 * Copyright (C) 2022 Adam Celarek <family name at cg tuwien ac at>
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

#include <catch2/catch.hpp>

#include "ctb/GlobalGeodetic.hpp"
#include "ctb/GlobalMercator.hpp"

// word of warning:
// the Grid class is taken from ctb, which didn't use unit tests.
// Therefore it's unlikely, that it will be fully covered by tests.
// New functions should be tested though!

TEST_CASE("grid")
{
    SECTION("epsg codes")
    {
        const auto geodetic = ctb::GlobalGeodetic(256);
        CHECK(geodetic.getEpsgCode() == 4326);

        const auto webmercator = ctb::GlobalMercator(256);
        CHECK(webmercator.getEpsgCode() == 3857);

        CHECK(!geodetic.getSRS().IsSame(&webmercator.getSRS()));
    }

    SECTION("number of tiles")
    {
        const auto geodetic = ctb::GlobalGeodetic(256);
        CHECK(geodetic.getTileExtent(0).getWidth() == 2);
        CHECK(geodetic.getTileExtent(0).getHeight() == 1);

        CHECK(geodetic.getTileExtent(1).getWidth() == 4);
        CHECK(geodetic.getTileExtent(1).getHeight() == 2);

        CHECK(geodetic.getTileExtent(2).getWidth() == 8);
        CHECK(geodetic.getTileExtent(2).getHeight() == 4);

        const auto webmercator = ctb::GlobalMercator(256);
        CHECK(webmercator.getTileExtent(0).getWidth() == 1);
        CHECK(webmercator.getTileExtent(0).getHeight() == 1);

        CHECK(webmercator.getTileExtent(1).getWidth() == 2);
        CHECK(webmercator.getTileExtent(1).getHeight() == 2);

        CHECK(webmercator.getTileExtent(2).getWidth() == 4);
        CHECK(webmercator.getTileExtent(2).getHeight() == 4);
    }

    SECTION("bounds")
    {
        const auto geodetic = ctb::GlobalGeodetic(256);
        CHECK(geodetic.getExtent().getMinX() == Approx(-180.0));
        CHECK(geodetic.getExtent().getMaxX() == Approx(180.0));
        CHECK(geodetic.getExtent().getMinY() == Approx(-90.0));
        CHECK(geodetic.getExtent().getMaxY() == Approx(90.0));

        const auto webmercator = ctb::GlobalMercator(256);
        const auto lim = double(20037508);
        CHECK(webmercator.getExtent().getMinX() == Approx(-lim));
        CHECK(webmercator.getExtent().getMaxX() == Approx(lim));
        CHECK(webmercator.getExtent().getMinY() == Approx(-lim));
        CHECK(webmercator.getExtent().getMaxY() == Approx(lim));
    }
}
