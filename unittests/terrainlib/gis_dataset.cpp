/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2022 Adam Celarek-Litofcenko
 * Copyright (C) 2022 Martin Braunsperger
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
#include "../temp_directory.h"

#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cpl_error.h>
#include <gdal_priv.h>

#include <array>
#include <filesystem>
#include <limits>
#include <optional>

#include "gis/CplError.h"
#include "gis/Dataset.h"
#include "gis/GeoTransform.h"
#include "gis/gdal.h"
#include "gis/srs/well_known.h"
#include "gis/webmercator.h"

using namespace radix;

void check_bounds(const radix::tile::SrsBounds &a, const radix::tile::SrsBounds &b) {
    CHECK(a.height() > 0);
    CHECK(a.width() > 0);
    CHECK(b.height() > 0);
    CHECK(b.width() > 0);

    // Reprojected coverage includes a guard band of 1/258 of the extent per side.
    CHECK_THAT(b.height(), Catch::Matchers::WithinRel(a.height(), 0.01));
    CHECK_THAT(b.width(), Catch::Matchers::WithinRel(a.width(), 0.01));
}

TEST_CASE("datasets are as expected", "[gis][Dataset]") {
    auto d_mgi = gis::Dataset(ALP_TEST_DATA_DIR "/austria/at_mgi.tif");
    auto d_wgs84 = gis::Dataset(ALP_TEST_DATA_DIR "/austria/at_wgs84.tif");

    SECTION("file name") {
        CHECK(d_mgi.name() == "at_mgi");
        CHECK(d_wgs84.name() == "at_wgs84");
    }

    SECTION("SRS") {
        const gis::srs::System mgi = REQUIRE_VALUE(d_mgi.srs());
        const gis::srs::System wgs84 = REQUIRE_VALUE(d_wgs84.srs());
        CHECK_FALSE(mgi.is_same(wgs84));
    }

    SECTION("coverage") {
        {
            const auto mgi_coverage = d_mgi.mercator_coverage().value();
            const auto wgs84_coverage = d_wgs84.mercator_coverage().value();
            REQUIRE(mgi_coverage.size() == 1);
            REQUIRE(wgs84_coverage.size() == 1);
            check_bounds(mgi_coverage.front(), wgs84_coverage.front());
        }
        {
            const auto mgi_coverage = d_mgi.geographic_coverage().value();
            const auto wgs84_coverage = d_wgs84.geographic_coverage().value();
            REQUIRE(mgi_coverage.size() == 1);
            REQUIRE(wgs84_coverage.size() == 1);
            check_bounds(mgi_coverage.front(), wgs84_coverage.front());
        }
    }
    SECTION("resolution") {
        CHECK(d_mgi.raster_size() == glm::uvec2(620, 350));
        CHECK(d_mgi.band_count() == 1);
    }
}

TEST_CASE("vector datasets use the layer spatial reference", "[gis][Dataset]") {
    gis::gdal::initialize_once();
    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("Memory");
    REQUIRE(driver != nullptr);

    GDALDataset *raw_dataset = driver->Create("", 0, 0, 0, GDT_Unknown, nullptr);
    REQUIRE(raw_dataset != nullptr);

    const gis::srs::System expected_srs = REQUIRE_VALUE(gis::srs::from_epsg(31287));
    REQUIRE(raw_dataset->CreateLayer("mask", &expected_srs.as_gdal(), wkbPolygon, nullptr) != nullptr);

    gis::Dataset dataset(raw_dataset);
    const gis::srs::System actual_srs = REQUIRE_VALUE(dataset.srs());
    CHECK(actual_srs.is_same(expected_srs));
}

namespace {
gis::Dataset memory_raster(std::array<double, 6> geotransform, glm::uvec2 size, int epsg, OSRAxisMappingStrategy axis_mapping = OAMS_TRADITIONAL_GIS_ORDER) {
    gis::gdal::initialize_once();
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("MEM");
    REQUIRE(driver != nullptr);
    gis::Dataset dataset(driver->Create("", int(size.x), int(size.y), 1, GDT_Float32, nullptr));
    REQUIRE(dataset.as_gdal()->SetGeoTransform(geotransform.data()) == CE_None);
    OGRSpatialReference reference;
    REQUIRE(reference.importFromEPSG(epsg) == OGRERR_NONE);
    // The axis mapping of a MEM raster cannot be changed once its system is set.
    reference.SetAxisMappingStrategy(axis_mapping);
    REQUIRE(dataset.as_gdal()->SetSpatialRef(&reference) == CE_None);
    return dataset;
}

// An in-memory raster of 4 by 2 pixels without georeferencing.
gis::Dataset make_empty_raster() {
    gis::gdal::initialize_once();
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("MEM");
    REQUIRE(driver != nullptr);
    return gis::Dataset(driver->Create("", 4, 2, 1, GDT_Float32, nullptr));
}

void check_bounds_near(const radix::tile::SrsBounds& actual, const radix::tile::SrsBounds& expected, double margin) {
    CHECK_THAT(actual.min.x, Catch::Matchers::WithinAbs(expected.min.x, margin));
    CHECK_THAT(actual.min.y, Catch::Matchers::WithinAbs(expected.min.y, margin));
    CHECK_THAT(actual.max.x, Catch::Matchers::WithinAbs(expected.max.x, margin));
    CHECK_THAT(actual.max.y, Catch::Matchers::WithinAbs(expected.max.y, margin));
}

glm::dvec2 mercator(double longitude, double latitude) {
    constexpr double radius = 6378137;
    return { radius * glm::radians(longitude), radius * std::log(std::tan(glm::radians(45 + latitude / 2))) };
}
} // namespace

TEST_CASE("srs fails as missing when neither the raster nor a layer has one", "[gis][Dataset]") {
    gis::Dataset dataset = make_empty_raster();
    dataset.as_gdal()->SetDescription("empty");

    const auto error = REQUIRE_ERROR(dataset.srs());

    CHECK(error.holds_code(gis::MissingSrs{"empty"}));
}

TEST_CASE("geo_transform reads the geotransform that was set", "[gis][Dataset]") {
    const auto dataset = memory_raster({ 10, 3, 1, 50, 2, -2 }, { 4, 2 }, 4326);

    const gis::GeoTransform transform = REQUIRE_VALUE(dataset.geo_transform());

    CHECK(transform == REQUIRE_VALUE(gis::GeoTransform::from_coefficients({ 10, 3, 1, 50, 2, -2 })));
}

TEST_CASE("geo_transform fails as missing for a raster without a geotransform", "[gis][Dataset]") {
    const gis::Dataset dataset = make_empty_raster();

    const auto error = REQUIRE_ERROR(dataset.geo_transform());

    CHECK(error.holds<gis::MissingGeoreferencing>());
}

TEST_CASE("geo_transform fails as invalid for a coefficient that is not finite", "[gis][Dataset]") {
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    const auto dataset = memory_raster({ 10, nan, 1, 50, 2, -2 }, { 4, 2 }, 4326);

    const auto error = REQUIRE_ERROR(dataset.geo_transform());

    CHECK(error.holds<gis::InvalidGeoreferencing>());
}

TEST_CASE("geo_transform fails for a system that orders its data axes north, east", "[gis][Dataset]") {
    const auto dataset = memory_raster({ 10, 3, 1, 50, 2, -2 }, { 4, 2 }, 4326, OAMS_AUTHORITY_COMPLIANT);

    const auto error = REQUIRE_ERROR(dataset.geo_transform());

    CHECK(error.holds<gis::srs::UnsupportedAxisOrder>());
}

TEST_CASE("bounds is the box around a north-up raster's corners", "[gis][Dataset]") {
    const auto dataset = memory_raster({ 10, 2, 0, 50, 0, -4 }, { 4, 2 }, 4326);

    const radix::tile::SrsBounds bounds = REQUIRE_VALUE(dataset.bounds());

    CHECK(bounds.min == glm::dvec2(10, 42));
    CHECK(bounds.max == glm::dvec2(18, 50));
}

TEST_CASE("bounds rejects a raster that is mirrored, south-up or sheared", "[gis][Dataset]") {
    std::array<double, 6> coefficients;
    SECTION("mirrored") { coefficients = { 10, -2, 0, 50, 0, -4 }; }
    SECTION("south up") { coefficients = { 10, 2, 0, 50, 0, 4 }; }
    SECTION("sheared") { coefficients = { 10, 3, 1, 50, 2, -2 }; }
    const auto dataset = memory_raster(coefficients, { 4, 2 }, 4326);

    const auto error = REQUIRE_ERROR(dataset.bounds());

    CHECK(error.holds<Unsupported>());
}

TEST_CASE("bounds rejects GCP, RPC and geolocation-array georeferencing", "[gis][Dataset]") {
    auto dataset = memory_raster({ 10, 0.1, 0, 50, 0, -0.1 }, { 32, 32 }, 4326);
    REQUIRE(dataset.bounds());
    SECTION("GCPs") {
        GDAL_GCP gcp {};
        REQUIRE(dataset.as_gdal()->SetGCPs(1, &gcp, dataset.as_gdal()->GetSpatialRef()) == CE_None);
    }
    SECTION("RPC") { REQUIRE(dataset.as_gdal()->SetMetadataItem("LINE_OFF", "1", "RPC") == CE_None); }
    SECTION("geolocation arrays") { REQUIRE(dataset.as_gdal()->SetMetadataItem("X_DATASET", "longitudes.tif", "GEOLOCATION") == CE_None); }

    const auto error = REQUIRE_ERROR(dataset.bounds());

    CHECK(error.holds<Unsupported>());
}

TEST_CASE("bounds3d contains every height of the band", "[gis][Dataset]") {
    auto dataset = memory_raster({ 10, 2, 0, 50, 0, -4 }, { 4, 2 }, 4326);
    GDALRasterBand *band = dataset.as_gdal()->GetRasterBand(1);
    std::array<float, 8> heights = { -3.5f, 0, 1, 2, 3, 4, 5, 12.25f };
    REQUIRE(band->RasterIO(GF_Write, 0, 0, 4, 2, heights.data(), 4, 2, GDT_Float32, 0, 0) == CE_None);
    SECTION("without statistics") {}
    SECTION("with computed statistics") { REQUIRE(band->ComputeStatistics(false, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr) == CE_None); }

    const radix::tile::SrsAndHeightBounds bounds = REQUIRE_VALUE(dataset.bounds3d());

    // The first pixel holds the lowest height and the last one the highest.
    CHECK(bounds.contains_inclusive(glm::dvec3(11, 48, -3.5)));
    CHECK(bounds.contains_inclusive(glm::dvec3(17, 44, 12.25)));
}

TEST_CASE("bounds3d accepts a band in metres or without a unit", "[gis][Dataset]") {
    auto dataset = memory_raster({ 10, 2, 0, 50, 0, -4 }, { 4, 2 }, 4326);
    SECTION("metres") { REQUIRE(dataset.as_gdal()->GetRasterBand(1)->SetUnitType("m") == CE_None); }
    SECTION("no unit") {}

    REQUIRE_VALUE(dataset.bounds3d());
}

TEST_CASE("bounds3d rejects a band whose height unit is not metres", "[gis][Dataset]") {
    auto dataset = memory_raster({ 10, 2, 0, 50, 0, -4 }, { 4, 2 }, 4326);
    REQUIRE(dataset.as_gdal()->GetRasterBand(1)->SetUnitType("ft") == CE_None);

    const auto error = REQUIRE_ERROR(dataset.bounds3d());

    CHECK(error.holds<Unsupported>());
}

TEST_CASE("dataset coverage is split at the antimeridian", "[gis][Dataset]") {
    const auto dataset = memory_raster({ 170, 0.625, 0, 5, 0, -0.3125 }, { 32, 32 }, 4326);

    const auto geographic = dataset.geographic_coverage();
    REQUIRE(geographic);
    REQUIRE(geographic->size() == 2);
    check_bounds_near((*geographic)[0], { { 170, -5 }, { 180, 5 } }, 1e-9);
    check_bounds_near((*geographic)[1], { { -180, -5 }, { -170, 5 } }, 1e-9);

    const auto mercator_bounds = dataset.mercator_coverage();
    REQUIRE(mercator_bounds);
    REQUIRE(mercator_bounds->size() == 2);
    check_bounds_near((*mercator_bounds)[0], { mercator(170, -5), mercator(180, 5) }, 1e-3);
    check_bounds_near((*mercator_bounds)[1], { mercator(-180, -5), mercator(-170, 5) }, 1e-3);
}

TEST_CASE("dataset coverage beyond the polar limit is empty in web mercator", "[gis][Dataset]") {
    const auto dataset = memory_raster({ 10, 0.1, 0, 89, 0, -0.02 }, { 32, 32 }, 4326);

    const auto geographic = dataset.geographic_coverage();
    REQUIRE(geographic);
    REQUIRE(geographic->size() == 1);
    check_bounds_near(geographic->front(), { { 10, 88.36 }, { 13.2, 89 } }, 1e-9);

    const auto mercator_bounds = dataset.mercator_coverage();
    REQUIRE(mercator_bounds);
    CHECK(mercator_bounds->empty());
}

TEST_CASE("web mercator dataset coverage is exact", "[gis][Dataset]") {
    // Tile (3, {4, 3}) in XYZ order.
    const double side = gis::webmercator::half_extent / 4;
    const auto dataset = memory_raster({ 0, side / 32, 0, side, 0, -side / 32 }, { 32, 32 }, 3857);

    const auto mercator_bounds = dataset.mercator_coverage();
    REQUIRE(mercator_bounds);
    REQUIRE(mercator_bounds->size() == 1);
    CHECK(mercator_bounds->front().min == glm::dvec2(0, 0));
    CHECK(mercator_bounds->front().max == glm::dvec2(side, side));
}

TEST_CASE("global dataset coverage is a single world rectangle", "[gis][Dataset]") {
    const auto dataset = memory_raster({ -180, 0.5, 0, 90, 0, -0.5 }, { 720, 360 }, 4326);

    const auto geographic = dataset.geographic_coverage();
    REQUIRE(geographic);
    REQUIRE(geographic->size() == 1);
    check_bounds_near(geographic->front(), { { -180, -90 }, { 180, 90 } }, 1e-9);

    const auto mercator_bounds = dataset.mercator_coverage();
    REQUIRE(mercator_bounds);
    REQUIRE(mercator_bounds->size() == 1);
    const double half = gis::webmercator::half_extent;
    check_bounds_near(mercator_bounds->front(), { { -half, -half }, { half, half } }, 1e-3);
}

TEST_CASE("pixel_transform fails as ambiguous for a raster with a second kind of georeferencing", "[gis][Dataset]") {
    auto dataset = memory_raster({ 10, 0.1, 0, 50, 0, -0.1 }, { 32, 32 }, 4326);
    REQUIRE(dataset.pixel_transform());
    SECTION("GCPs") {
        GDAL_GCP gcp {};
        REQUIRE(dataset.as_gdal()->SetGCPs(1, &gcp, dataset.as_gdal()->GetSpatialRef()) == CE_None);
    }
    SECTION("RPC") { REQUIRE(dataset.as_gdal()->SetMetadataItem("LINE_OFF", "1", "RPC") == CE_None); }
    SECTION("geolocation arrays") { REQUIRE(dataset.as_gdal()->SetMetadataItem("X_DATASET", "longitudes.tif", "GEOLOCATION") == CE_None); }
    const auto error = REQUIRE_ERROR(dataset.pixel_transform());
    CHECK(error.holds<gis::AmbiguousGeoreferencing>());
}

TEST_CASE("pixel_transform fails as missing for a kind the raster does not have", "[gis][Dataset]") {
    const auto dataset = memory_raster({ 10, 0.1, 0, 50, 0, -0.1 }, { 32, 32 }, 4326);

    const auto error = REQUIRE_ERROR(dataset.pixel_transform(gis::GeoreferencingKind::GcpPolynomial));

    CHECK(error.holds<gis::MissingGeoreferencing>());
}

TEST_CASE("pixel_transform fails for a system that orders its data axes north, east", "[gis][Dataset]") {
    const auto dataset = memory_raster({ 10, 0.1, 0, 50, 0, -0.1 }, { 32, 32 }, 4326, OAMS_AUTHORITY_COMPLIANT);

    const auto error = REQUIRE_ERROR(dataset.pixel_transform(gis::GeoreferencingKind::GeoTransform));

    CHECK(error.holds<gis::srs::UnsupportedAxisOrder>());
}

TEST_CASE("georeferencing_kind is the only kind of georeferencing a raster has", "[gis][Dataset]") {
    SECTION("geotransform") {
        const auto dataset = memory_raster({ 10, 0.1, 0, 50, 0, -0.1 }, { 32, 32 }, 4326);

        CHECK(REQUIRE_VALUE(dataset.georeferencing_kind()) == gis::GeoreferencingKind::GeoTransform);
    }
    SECTION("RPC") {
        gis::Dataset dataset = make_empty_raster();
        REQUIRE(dataset.as_gdal()->SetMetadataItem("LINE_OFF", "1", "RPC") == CE_None);

        CHECK(REQUIRE_VALUE(dataset.georeferencing_kind()) == gis::GeoreferencingKind::Rpc);
    }
}

TEST_CASE("georeferencing_kind fails as missing for a raster without georeferencing", "[gis][Dataset]") {
    const gis::Dataset dataset = make_empty_raster();

    const auto error = REQUIRE_ERROR(dataset.georeferencing_kind());

    CHECK(error.holds<gis::MissingGeoreferencing>());
}

TEST_CASE("georeferencing_kind fails as ambiguous for a raster with a second kind of georeferencing", "[gis][Dataset]") {
    auto dataset = memory_raster({ 10, 0.1, 0, 50, 0, -0.1 }, { 32, 32 }, 4326);
    SECTION("GCPs") {
        GDAL_GCP gcp {};
        REQUIRE(dataset.as_gdal()->SetGCPs(1, &gcp, dataset.as_gdal()->GetSpatialRef()) == CE_None);
    }
    SECTION("RPC") { REQUIRE(dataset.as_gdal()->SetMetadataItem("LINE_OFF", "1", "RPC") == CE_None); }
    SECTION("geolocation arrays") { REQUIRE(dataset.as_gdal()->SetMetadataItem("X_DATASET", "longitudes.tif", "GEOLOCATION") == CE_None); }

    const auto error = REQUIRE_ERROR(dataset.georeferencing_kind());

    CHECK(error.holds<gis::AmbiguousGeoreferencing>());
}

TEST_CASE("open_raster, open_vector and open_shared_raster keep GDAL's message for a missing path", "[gis][Dataset]") {
    const std::filesystem::path path = ALP_TEST_DATA_DIR "/missing.tif";
    std::optional<gis::CplError> error;

    SECTION("raster") {
        const auto result = gis::Dataset::open_raster(path);
        REQUIRE(!result.has_value());
        error = result.error().code();
    }

    SECTION("vector") {
        const auto result = gis::Dataset::open_vector(path);
        REQUIRE(!result.has_value());
        error = result.error().code();
    }

    SECTION("shared raster") {
        const auto result = gis::Dataset::open_shared_raster(path);
        REQUIRE(!result.has_value());
        error = result.error().code();
    }

    CPLErrorReset();
    CHECK(error->message() != gis::CplError::get_last_error().message());
}

TEST_CASE("reopen opens the dataset again as the same kind", "[gis][Dataset]") {
    gis::gdal::initialize_once();
    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("GPKG");
    REQUIRE(driver != nullptr);
    const TempDirectory directory;
    const std::filesystem::path path = directory.path() / "raster_and_layer.gpkg";
    {
        // A GeoPackage registers its raster table once the raster has a geotransform and a system.
        gis::Dataset raster(driver->Create(path.c_str(), 4, 4, 1, GDT_Byte, nullptr));
        std::array<double, 6> geotransform{0, 1, 0, 4, 0, -1};
        REQUIRE(raster.as_gdal()->SetGeoTransform(geotransform.data()) == CE_None);
        REQUIRE(raster.as_gdal()->SetSpatialRef(&gis::srs::webmercator().as_gdal()) == CE_None);
    }
    {
        gis::Dataset layers(GDALDataset::Open(path.c_str(), GDAL_OF_VECTOR | GDAL_OF_UPDATE));
        REQUIRE(layers.as_gdal()->CreateLayer("points", nullptr, wkbPoint, nullptr) != nullptr);
    }

    SECTION("raster") {
        const gis::Dataset original = REQUIRE_VALUE(gis::Dataset::open_raster(path));
        gis::Dataset reopened = REQUIRE_VALUE(original.reopen());
        CHECK(reopened.band_count() != 0);
        CHECK(reopened.as_gdal()->GetLayerCount() == 0);
    }

    SECTION("vector") {
        const gis::Dataset original = REQUIRE_VALUE(gis::Dataset::open_vector(path));
        gis::Dataset reopened = REQUIRE_VALUE(original.reopen());
        CHECK(reopened.band_count() == 0);
        CHECK(reopened.as_gdal()->GetLayerCount() == 1);
    }
}
