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
#include "gis/srs/coverage.h"
#include "gis/srs/Transform.h"
#include "gis/srs/well_known.h"
#include "gis/webmercator.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <ogr_spatialref.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace {
using Bounds = radix::geometry::Aabb3d;
using Catch::Approx;

// Regular samples of a box, including its interior.
std::vector<glm::dvec3> box_samples(const Bounds& bounds)
{
    constexpr int horizontal = 40;
    constexpr int vertical = 4;
    std::vector<glm::dvec3> points;
    for (int z = 0; z <= vertical; ++z) {
        for (int y = 0; y <= horizontal; ++y) {
            for (int x = 0; x <= horizontal; ++x) {
                points.push_back(glm::mix(bounds.min, bounds.max, glm::dvec3(x, y, 0) / double(horizontal) + glm::dvec3(0, 0, z / double(vertical))));
            }
        }
    }
    return points;
}

Bounds bounds_of(const std::vector<glm::dvec3>& points)
{
    Bounds bounds;
    for (const auto& point : points) {
        bounds.expand_by(point);
    }
    return bounds;
}

size_t count_uncovered(const std::vector<Bounds>& coverage, const std::vector<glm::dvec3>& points)
{
    return std::ranges::count_if(points,
        [&](const glm::dvec3& point) { return std::ranges::none_of(coverage, [&](const Bounds& bounds) { return bounds.contains_inclusive(point); }); });
}

glm::dvec3 ecef_point(double longitude, double latitude, double height)
{
    return REQUIRE_VALUE(gis::srs::transform_point(gis::srs::wgs84(), gis::srs::ecef(), glm::dvec3(longitude, latitude, height)));
}

Bounds cube(const glm::dvec3& centre, double half_size) { return { centre - half_size, centre + half_size }; }

struct Reference {
    std::string name;
    gis::srs::System srs;
};
} // namespace

TEST_CASE("ECEF coverage encloses the transformed box", "[gis][srs][coverage]")
{
    struct Case {
        std::string name;
        gis::srs::System srs;
        Bounds bounds;
    };
    const std::vector<Case> cases {
        { "geographic_1km", gis::srs::wgs84(), { { 16.36, 48.20, 0 }, { 16.37, 48.21, 1000 } } },
        { "austria", gis::srs::wgs84(), { { 9.5, 46.3, -100 }, { 17.2, 49.1, 4000 } } },
        { "equator", gis::srs::wgs84(), { { -0.37, -0.28, 0 }, { 0.59, 0.44, 1000 } } },
        { "near_pole", gis::srs::wgs84(), { { 10, 89.0, 0 }, { 20, 89.9, 500 } } },
        { "antimeridian", gis::srs::wgs84(), { { 170, -5, 0 }, { 190, 5, 100 } } },
        { "mercator_10km", gis::srs::webmercator(), { { 1810000, 6130000, 0 }, { 1820000, 6140000, 1000 } } },
        { "austria_lambert", REQUIRE_VALUE(gis::srs::from_epsg(31287)), { { 400000, 400000, 0 }, { 500000, 480000, 3000 } } },
        { "utm_33n", REQUIRE_VALUE(gis::srs::from_epsg(32633)), { { 500000, 5300000, 0 }, { 600000, 5400000, 3000 } } },
    };
    for (const auto& scenario : cases) {
        CAPTURE(scenario.name);
        const auto coverage = REQUIRE_VALUE(gis::srs::coverage::compute_ecef(scenario.srs, scenario.bounds));
        const auto samples = REQUIRE_VALUE(gis::srs::transform_points(scenario.srs, gis::srs::ecef(), box_samples(scenario.bounds)));
        CHECK(count_uncovered({ coverage }, samples) == 0);
        // Going through a longitude/latitude box loosens projected boxes by their rotation
        // against the meridians, a few percent here.
        const auto sampled = bounds_of(samples);
        CAPTURE(coverage.size() / sampled.size());
        CHECK(glm::all(glm::lessThanEqual(coverage.size(), sampled.size() * 1.05 + 0.01)));
    }
}

TEST_CASE("ECEF coverage of geographic boxes is exact", "[gis][srs][coverage]")
{
    SECTION("extrema at corners")
    {
        const Bounds austria { { 9.5, 46.3, -100 }, { 17.2, 49.1, 4000 } };
        Bounds corners;
        for (const double longitude : { austria.min.x, austria.max.x }) {
            for (const double latitude : { austria.min.y, austria.max.y }) {
                for (const double height : { austria.min.z, austria.max.z }) {
                    corners.expand_by(ecef_point(longitude, latitude, height));
                }
            }
        }
        const auto coverage = REQUIRE_VALUE(gis::srs::coverage::compute_ecef(gis::srs::wgs84(), austria));
        for (int axis = 0; axis < 3; ++axis) {
            CHECK(coverage.min[axis] == Approx(corners.min[axis]).margin(0.01));
            CHECK(coverage.max[axis] == Approx(corners.max[axis]).margin(0.01));
        }
    }
    SECTION("extremum inside the top face, at the equator and the prime meridian")
    {
        const auto coverage = REQUIRE_VALUE(gis::srs::coverage::compute_ecef(gis::srs::wgs84(), { { -0.37, -0.28, 0 }, { 0.59, 0.44, 1000 } }));
        CHECK(coverage.max.x == Approx(6378137.0 + 1000).margin(0.01));
    }
    SECTION("extremum inside an edge, at the prime meridian")
    {
        const auto coverage = REQUIRE_VALUE(gis::srs::coverage::compute_ecef(gis::srs::wgs84(), { { -1, 10, 0 }, { 1, 11, 100 } }));
        CHECK(coverage.max.x == Approx(ecef_point(0, 10, 100).x).margin(0.01));
    }
}

TEST_CASE("Coverage of ECEF boxes encloses the transformed box", "[gis][srs][coverage]")
{
    const auto vienna = ecef_point(16.37, 48.21, 250);
    const std::vector<Reference> references {
        { "wgs84", gis::srs::wgs84() },
        { "webmercator", gis::srs::webmercator() },
        { "mgi", gis::srs::mgi() },
        { "austria_lambert", REQUIRE_VALUE(gis::srs::from_epsg(31287)) },
        { "utm_33n", REQUIRE_VALUE(gis::srs::from_epsg(32633)) },
    };
    for (const auto& [box_name, box] :
        { std::pair { "1km", cube(vienna, 500) }, std::pair { "100km", cube(vienna, 50000) }, std::pair { "1000km", cube(vienna, 500000) } }) {
        for (const auto& reference : references) {
            CAPTURE(box_name, reference.name);
            const auto coverage = REQUIRE_VALUE(gis::srs::coverage::compute_from_ecef(box, reference.srs));
            REQUIRE(coverage.size() == 1);
            const auto samples = REQUIRE_VALUE(gis::srs::transform_points(gis::srs::ecef(), reference.srs, box_samples(box)));
            CHECK(count_uncovered(coverage, samples) == 0);
            // The longitude/latitude box is rotated against the projected axes by the meridian
            // convergence, about 2 degrees at Vienna for UTM 33N and Austria Lambert.
            const auto sampled = bounds_of(samples);
            const auto looseness = (coverage.front().size() - sampled.size()) / sampled.size();
            CAPTURE(looseness);
            CHECK(glm::all(glm::lessThanEqual(looseness, glm::dvec3(0.1))));
        }
    }
}

TEST_CASE("Coverage of ECEF boxes is exact in WGS84", "[gis][srs][coverage]")
{
    SECTION("extrema at corners")
    {
        const auto box = cube(ecef_point(16.37, 48.21, 250), 50000);
        const auto coverage = REQUIRE_VALUE(gis::srs::coverage::compute_from_ecef(box, gis::srs::wgs84()));
        REQUIRE(coverage.size() == 1);
        // The extrema are at corners of the box, which are among the samples.
        const auto sampled = bounds_of(REQUIRE_VALUE(gis::srs::transform_points(gis::srs::ecef(), gis::srs::wgs84(), box_samples(box))));
        for (int axis = 0; axis < 3; ++axis) {
            const double margin = axis < 2 ? 1e-8 : 0.01;
            CHECK(coverage.front().min[axis] == Approx(sampled.min[axis]).margin(margin));
            CHECK(coverage.front().max[axis] == Approx(sampled.max[axis]).margin(margin));
        }
    }
    SECTION("height minimum inside a face, at the equator")
    {
        const auto coverage = REQUIRE_VALUE(gis::srs::coverage::compute_from_ecef(cube({ 6378137, 0, 0 }, 50000), gis::srs::wgs84()));
        REQUIRE(coverage.size() == 1);
        CHECK(coverage.front().min.z == Approx(-50000).margin(0.01));
    }
}

TEST_CASE("Coverage of ECEF boxes stays within the bounds of the SRS", "[gis][srs][coverage]")
{
    SECTION("antimeridian")
    {
        const auto box = cube({ -6378137, 0, 0 }, 50000);
        for (const auto& reference : { gis::srs::wgs84(), gis::srs::webmercator() }) {
            const auto coverage = REQUIRE_VALUE(gis::srs::coverage::compute_from_ecef(box, reference));
            REQUIRE(coverage.size() == 2);
            const double half_width = reference.kind() == gis::srs::Kind::Geographic ? 180 : gis::webmercator::half_extent;
            for (const auto& bounds : coverage) {
                CHECK(bounds.min.x >= -half_width);
                CHECK(bounds.max.x <= half_width);
            }
            const auto samples = REQUIRE_VALUE(gis::srs::transform_points(gis::srs::ecef(), reference, box_samples(box)));
            CHECK(count_uncovered(coverage, samples) == 0);
        }
    }
    SECTION("north pole")
    {
        const auto box = cube({ 0, 0, 6356752.3 }, 50000);
        const auto geographic = REQUIRE_VALUE(gis::srs::coverage::compute_from_ecef(box, gis::srs::wgs84()));
        REQUIRE(geographic.size() == 1);
        CHECK(geographic.front().min.x == -180);
        CHECK(geographic.front().max.x == 180);
        CHECK(geographic.front().max.y == 90);
        CHECK(count_uncovered(geographic, REQUIRE_VALUE(gis::srs::transform_points(gis::srs::ecef(), gis::srs::wgs84(), box_samples(box)))) == 0);
        const auto mercator = REQUIRE_VALUE(gis::srs::coverage::compute_from_ecef(box, gis::srs::webmercator()));
        CHECK(mercator.empty());
    }
    SECTION("centre of the earth")
    {
        const auto error = REQUIRE_ERROR(gis::srs::coverage::compute_from_ecef(cube(glm::dvec3(0), 1000), gis::srs::wgs84()));
        CHECK(error.holds_code(gis::srs::coverage::UnsupportedBounds::NearEarthCentre));
    }
}

TEST_CASE("Coverage allows for height-dependent datum shifts", "[gis][srs][coverage]")
{
    // MGI shifts by about 0.3 m between heights 0 and 20 km at Vienna. Degenerate boxes in
    // Austria Lambert get a tiny guard band, which would hide less than that.
    const auto lambert = REQUIRE_VALUE(gis::srs::from_epsg(31287));
    for (const double height : { -20000.0, 20000.0 }) {
        CAPTURE(height);
        const auto ecef = ecef_point(16.37, 48.21, height);
        const auto projected = REQUIRE_VALUE(gis::srs::transform_point(gis::srs::ecef(), lambert, ecef));
        const auto backward = REQUIRE_VALUE(gis::srs::coverage::compute_from_ecef({ ecef, ecef }, lambert));
        CHECK(count_uncovered(backward, { projected }) == 0);
        const auto forward = REQUIRE_VALUE(gis::srs::coverage::compute_ecef(lambert, { projected, projected }));
        CHECK(forward.contains_inclusive(REQUIRE_VALUE(gis::srs::transform_point(lambert, gis::srs::ecef(), projected))));
    }
}

TEST_CASE("Coverage functions handle ECEF and compound SRSes", "[gis][srs][coverage]")
{
    const Bounds box = cube(ecef_point(16.37, 48.21, 250), 500);
    CHECK(REQUIRE_VALUE(gis::srs::coverage::compute_ecef(gis::srs::ecef(), box)) == box);
    const auto unchanged = REQUIRE_VALUE(gis::srs::coverage::compute_from_ecef(box, gis::srs::ecef()));
    REQUIRE(unchanged.size() == 1);
    CHECK(unchanged.front() == box);

    const auto compound = REQUIRE_VALUE(gis::srs::from_user_input("EPSG:4326+3855"));
    REQUIRE(compound.is_compound());
    const auto forward = REQUIRE_ERROR(gis::srs::coverage::compute_ecef(compound, { { 16.36, 48.20, 0 }, { 16.37, 48.21, 1000 } }));
    CHECK(forward.holds<gis::srs::coverage::CompoundSystem>());
    const auto backward = REQUIRE_ERROR(gis::srs::coverage::compute_from_ecef(box, compound));
    CHECK(backward.holds<gis::srs::coverage::CompoundSystem>());
}

TEST_CASE("compute_from_ecef rejects systems it cannot cover", "[gis][srs][coverage]")
{
    const Bounds box = cube(ecef_point(16.37, 48.21, 250), 500);
    SECTION("a system that is not geodetic")
    {
        const auto vertical = REQUIRE_VALUE(gis::srs::from_epsg(5773));
        const auto error = REQUIRE_ERROR(gis::srs::coverage::compute_from_ecef(box, vertical));
        CHECK(error.holds<gis::srs::NotGeodetic>());
    }
    SECTION("a geographic system in grads")
    {
        const auto grads = REQUIRE_VALUE(gis::srs::from_epsg(4807));
        const auto error = REQUIRE_ERROR(gis::srs::coverage::compute_from_ecef(box, grads));
        CHECK(error.holds<gis::srs::coverage::AnglesNotDegrees>());
    }
    SECTION("a geographic system whose latitude points south")
    {
        const auto south = REQUIRE_VALUE(gis::srs::from_user_input(R"(GEOGCRS["south latitude",DATUM["World Geodetic System 1984",ELLIPSOID["WGS 84",6378137,298.257223563]],CS[ellipsoidal,2],AXIS["longitude",east,ANGLEUNIT["degree",0.0174532925199433]],AXIS["latitude",south,ANGLEUNIT["degree",0.0174532925199433]]])"));
        const auto error = REQUIRE_ERROR(gis::srs::coverage::compute_from_ecef(box, south));
        CHECK(error.holds<gis::srs::UnsupportedAxisOrder>());
    }
}

TEST_CASE("wrap_longitudes shifts bounds into the half circle around the longitude of the centre", "[gis][srs][coverage]")
{
    const auto wrapped = REQUIRE_VALUE(gis::srs::coverage::wrap_longitudes(gis::srs::wgs84(), { { -170, -5 }, { -160, 5 } }, { 185, 0 }));

    CHECK_THAT(wrapped, UnorderedWithinEpsilon(std::vector<radix::tile::SrsBounds> { { { 190, -5 }, { 200, 5 } } }));
}

TEST_CASE("wrap_longitudes wraps the longitude wherever it is among the data axes", "[gis][srs][coverage]")
{
    OGRSpatialReference authority_srs;
    REQUIRE(authority_srs.importFromEPSG(4326) == OGRERR_NONE);
    const gis::srs::System latitude_first(std::move(authority_srs));

    const auto wrapped = REQUIRE_VALUE(gis::srs::coverage::wrap_longitudes(latitude_first, { { -5, -170 }, { 5, -160 } }, { 0, 185 }));

    CHECK_THAT(wrapped, UnorderedWithinEpsilon(std::vector<radix::tile::SrsBounds> { { { -5, 190 }, { 5, 200 } } }));
}

TEST_CASE("wrap_longitudes wraps by a full circle in the unit of the longitude", "[gis][srs][coverage]")
{
    const auto grads = REQUIRE_VALUE(gis::srs::from_user_input(
        R"(GEOGCRS["grads",DATUM["World Geodetic System 1984",ELLIPSOID["WGS 84",6378137,298.257223563]],CS[ellipsoidal,2],AXIS["longitude",east,ANGLEUNIT["grad",0.015707963267949]],AXIS["latitude",north,ANGLEUNIT["grad",0.015707963267949]]])"));

    const auto wrapped = REQUIRE_VALUE(gis::srs::coverage::wrap_longitudes(grads, { { -210, 10 }, { -200, 20 } }, { 200, 0 }));

    CHECK_THAT(wrapped, UnorderedWithinEpsilon(std::vector<radix::tile::SrsBounds> { { { 190, 10 }, { 200, 20 } } }));
}

TEST_CASE("wrap_longitudes fails for a system that is not geographic", "[gis][srs][coverage]")
{
    const auto error = REQUIRE_ERROR(gis::srs::coverage::wrap_longitudes(gis::srs::webmercator(), { { -170, -5 }, { -160, 5 } }, { 185, 0 }));

    CHECK(error.holds<gis::srs::coverage::NotGeographic>());
}

// TODO: enable once gis::srs::System reads per-axis units; System::axes() reports the metre axes of a geographic system as radians (see the merge todo).
/*
TEST_CASE("wrap_longitudes fails without an angular longitude", "[gis][srs][coverage]")
{
    const auto metres = REQUIRE_VALUE(gis::srs::from_user_input(
        R"(GEOGCRS["metres",DATUM["World Geodetic System 1984",ELLIPSOID["WGS 84",6378137,298.257223563]],CS[ellipsoidal,2],AXIS["longitude",east,LENGTHUNIT["metre",1]],AXIS["latitude",north,LENGTHUNIT["metre",1]]])"));

    const auto error = REQUIRE_ERROR(gis::srs::coverage::wrap_longitudes(metres, { { -170, -5 }, { -160, 5 } }, { 185, 0 }));

    CHECK(error.holds<gis::srs::UnsupportedAxisOrder>());
}
*/
