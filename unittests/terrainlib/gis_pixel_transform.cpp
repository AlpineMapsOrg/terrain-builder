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

#include "../catch2_helpers.h"
#include "../result_helpers.h"

#include <array>
#include <limits>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <gdal_priv.h>
#include <glm/glm.hpp>

#include "gis/Dataset.h"
#include "gis/PixelTransform.h"
#include "gis/gdal.h"
#include "gis/georeferencing.h"
#include "gis/srs/System.h"

namespace {
// An in-memory raster of 32 by 32 pixels without georeferencing.
gis::Dataset make_bare_raster() {
    gis::gdal::initialize_once();
    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("MEM");
    REQUIRE(driver != nullptr);
    return gis::Dataset(driver->Create("", 32, 32, 1, GDT_Float32, nullptr));
}

// An in-memory raster of 32 by 32 pixels with `coefficients` as its geotransform in WGS84.
gis::Dataset make_raster(std::array<double, 6> coefficients) {
    gis::Dataset dataset = make_bare_raster();
    REQUIRE(dataset.as_gdal()->SetGeoTransform(coefficients.data()) == CE_None);
    const gis::srs::System wgs84 = REQUIRE_VALUE(gis::srs::from_epsg(4326));
    REQUIRE(dataset.as_gdal()->SetSpatialRef(&wgs84.as_gdal()) == CE_None);
    return dataset;
}
} // namespace

TEST_CASE("make_pixel_transform maps pixels by the georeferencing of the given kind", "[gis][PixelTransform]") {
    SECTION("geotransform and GCPs") {
        // The GCPs put the raster one degree further east and north than its geotransform does.
        gis::Dataset dataset = make_raster({10, 0.1, 0, 50, 0, -0.1});
        const std::array<GDAL_GCP, 4> gcps{{
            {nullptr, nullptr, 0, 0, 11, 51, 0},
            {nullptr, nullptr, 32, 0, 14.2, 51, 0},
            {nullptr, nullptr, 0, 32, 11, 47.8, 0},
            {nullptr, nullptr, 32, 32, 14.2, 47.8, 0},
        }};
        REQUIRE(dataset.as_gdal()->SetGCPs(static_cast<int>(gcps.size()), gcps.data(), dataset.as_gdal()->GetSpatialRef()) == CE_None);
        SECTION("geotransform") {
            const gis::PixelTransform transform = REQUIRE_VALUE(gis::make_pixel_transform(dataset, gis::GeoreferencingKind::GeoTransform));

            const glm::dvec2 point = REQUIRE_VALUE(transform.apply(glm::dvec2(4, 2)));

            CHECK_THAT(point.x, WithinEpsilon(10.4));
            CHECK_THAT(point.y, WithinEpsilon(49.8));
        }
        SECTION("GCPs") {
            const gis::PixelTransform transform = REQUIRE_VALUE(gis::make_pixel_transform(dataset, gis::GeoreferencingKind::GcpPolynomial));

            const glm::dvec2 point = REQUIRE_VALUE(transform.apply(glm::dvec2(4, 2)));

            CHECK_THAT(point.x, WithinEpsilon(11.4));
            CHECK_THAT(point.y, WithinEpsilon(50.8));
        }
    }
    SECTION("RPC") {
        // RPC offsets refer to pixel centres (GDAL RFC 22), so column 50 is at x 50.5, 10 degrees east, and row 50 at y 50.5, 47 degrees north, at 50 pixels per degree, with rows running south.
        gis::Dataset dataset = make_bare_raster();
        const std::string zeros = " 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0";
        REQUIRE(dataset.as_gdal()->SetMetadataItem("LINE_OFF", "50", "RPC") == CE_None);
        REQUIRE(dataset.as_gdal()->SetMetadataItem("SAMP_OFF", "50", "RPC") == CE_None);
        REQUIRE(dataset.as_gdal()->SetMetadataItem("LAT_OFF", "47", "RPC") == CE_None);
        REQUIRE(dataset.as_gdal()->SetMetadataItem("LONG_OFF", "10", "RPC") == CE_None);
        REQUIRE(dataset.as_gdal()->SetMetadataItem("HEIGHT_OFF", "0", "RPC") == CE_None);
        REQUIRE(dataset.as_gdal()->SetMetadataItem("LINE_SCALE", "50", "RPC") == CE_None);
        REQUIRE(dataset.as_gdal()->SetMetadataItem("SAMP_SCALE", "50", "RPC") == CE_None);
        REQUIRE(dataset.as_gdal()->SetMetadataItem("LAT_SCALE", "1", "RPC") == CE_None);
        REQUIRE(dataset.as_gdal()->SetMetadataItem("LONG_SCALE", "1", "RPC") == CE_None);
        REQUIRE(dataset.as_gdal()->SetMetadataItem("HEIGHT_SCALE", "1", "RPC") == CE_None);
        // The terms are 1, longitude, latitude, height and higher orders.
        REQUIRE(dataset.as_gdal()->SetMetadataItem("LINE_NUM_COEFF", ("0 0 -1" + zeros).c_str(), "RPC") == CE_None);
        REQUIRE(dataset.as_gdal()->SetMetadataItem("LINE_DEN_COEFF", ("1 0 0" + zeros).c_str(), "RPC") == CE_None);
        REQUIRE(dataset.as_gdal()->SetMetadataItem("SAMP_NUM_COEFF", ("0 1 0" + zeros).c_str(), "RPC") == CE_None);
        REQUIRE(dataset.as_gdal()->SetMetadataItem("SAMP_DEN_COEFF", ("1 0 0" + zeros).c_str(), "RPC") == CE_None);
        const gis::PixelTransform transform = REQUIRE_VALUE(gis::make_pixel_transform(dataset, gis::GeoreferencingKind::Rpc));

        const glm::dvec2 point = REQUIRE_VALUE(transform.apply(glm::dvec2(75.5, 25.5)));
        const glm::dvec2 pixel = REQUIRE_VALUE(transform.inverse().apply(point));

        // GDAL maps pixels to the ground iteratively, to within its documented RPC_PIXEL_ERROR_THRESHOLD of 0.1 pixels.
        CHECK_THAT(point.x, Catch::Matchers::WithinAbs(10.5, 0.1 / 50));
        CHECK_THAT(point.y, Catch::Matchers::WithinAbs(47.5, 0.1 / 50));
        CHECK_THAT(pixel.x, Catch::Matchers::WithinAbs(75.5, 0.1));
        CHECK_THAT(pixel.y, Catch::Matchers::WithinAbs(25.5, 0.1));
    }
}

TEST_CASE("make_pixel_transform fails as invalid for a geotransform that is not finite", "[gis][PixelTransform]") {
    const gis::Dataset dataset = make_raster({10, std::numeric_limits<double>::quiet_NaN(), 0, 50, 0, -0.1});

    const auto error = REQUIRE_ERROR(gis::make_pixel_transform(dataset, gis::GeoreferencingKind::GeoTransform));

    CHECK(error.holds<gis::InvalidGeoreferencing>());
}

TEST_CASE("make_pixel_transform fails as invalid when GDAL cannot build the transformer", "[gis][PixelTransform]") {
    // A polynomial needs at least three GCPs.
    gis::Dataset dataset = make_bare_raster();
    const GDAL_GCP gcp{nullptr, nullptr, 0, 0, 10, 50, 0};
    const gis::srs::System wgs84 = REQUIRE_VALUE(gis::srs::from_epsg(4326));
    REQUIRE(dataset.as_gdal()->SetGCPs(1, &gcp, &wgs84.as_gdal()) == CE_None);

    const auto error = REQUIRE_ERROR(gis::make_pixel_transform(dataset, gis::GeoreferencingKind::GcpPolynomial));

    CHECK(error.holds<gis::InvalidGeoreferencing>());
}

TEST_CASE("inverse maps the image back and keeps the kind", "[gis][PixelTransform]") {
    const gis::Dataset dataset = make_raster({10, 0.1, 0, 50, 0, -0.1});
    const gis::PixelTransform transform = REQUIRE_VALUE(gis::make_pixel_transform(dataset, gis::GeoreferencingKind::GeoTransform));
    const glm::dvec2 point = REQUIRE_VALUE(transform.apply(glm::dvec2(4, 2)));

    const gis::PixelTransform inverse = transform.inverse();
    const glm::dvec2 pixel = REQUIRE_VALUE(inverse.apply(point));

    CHECK_THAT(pixel.x, WithinEpsilon(4));
    CHECK_THAT(pixel.y, WithinEpsilon(2));
    CHECK(inverse.kind() == gis::GeoreferencingKind::GeoTransform);
}
