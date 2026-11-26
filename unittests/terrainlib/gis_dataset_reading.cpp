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

#include <algorithm>
#include <array>
#include <cfloat>
#include <climits>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <numbers>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <fmt/core.h>
#include <gdal_priv.h>

#include "../assertion_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "gis/Dataset.h"
#include "gis/DatasetReader.h"
#include "gis/gdal.h"
#include "gis/srs/System.h"
#include "gis/srs/Transform.h"
#include "gis/srs/well_known.h"
#include "gis/webmercator.h"
#include "io/image.h"

using namespace radix;

namespace {
// Writes the raster as a grayscale PNG, scaled from its minimum to its maximum.
template <typename T>
void debug_out(const radix::Raster<T> &image, const std::filesystem::path &path) {
    REQUIRE(!image.buffer().empty());
    const auto [min, max] = std::ranges::minmax(image);
    const float range = static_cast<float>(max) - static_cast<float>(min);
    const auto gray = radix::raster::transform(image, [min, range](const auto value) {
        const uint8_t intensity = range == 0.F ? 0 : static_cast<uint8_t>(255.F * (static_cast<float>(value) - static_cast<float>(min)) / range);
        return glm::u8vec3(intensity);
    });
    REQUIRE_VALUE(io::image::write(gray, path, {.mode = io::WriteMode::Overwrite}));
}

void require_projection_available(const gis::Dataset &dataset, const gis::srs::System &target_srs) {
    const gis::srs::System dataset_srs = REQUIRE_VALUE(dataset.srs());
    const radix::tile::SrsBounds source_bounds = REQUIRE_VALUE(dataset.bounds());
    const gis::srs::Transform transform = REQUIRE_VALUE(gis::srs::get_transform(dataset_srs, target_srs));
    REQUIRE(transform.apply((source_bounds.min + source_bounds.max) / 2.0).has_value());
}

template <typename Pixel>
gis::DatasetReader<Pixel> make_reader(std::shared_ptr<gis::Dataset> dataset, const gis::Projection projection, const std::array<uint32_t, gis::DatasetReader<Pixel>::channel_count> bands, const Pixel default_pixel) {
    return REQUIRE_VALUE(gis::DatasetReader<Pixel>::make(std::move(dataset), projection, bands, default_pixel));
}

template <typename Pixel>
typename gis::DatasetReader<Pixel>::Samples read_samples(gis::DatasetReader<Pixel> &reader, const radix::tile::SrsBounds &bounds, const glm::uvec2 size) {
    auto samples = REQUIRE_VALUE(reader.read(bounds, size));
    REQUIRE(samples.data.size() == size);
    REQUIRE(samples.valid.size() == size);
    return samples;
}

bool all_valid(const radix::Raster<uint8_t> &valid) {
    return std::ranges::all_of(valid, [](const uint8_t value) { return value != 0; });
}

std::array<double, 6> geo_transform_for(const radix::tile::SrsBounds &bounds, const glm::uvec2 size) {
    return {bounds.min.x, bounds.width() / size.x, 0, bounds.max.y, 0, -bounds.height() / size.y};
}

// A raster without georeferencing, whose bands are each filled with 10 times their number.
// An empty path creates an in-memory raster, otherwise a GeoTIFF.
std::shared_ptr<gis::Dataset> make_bare_raster(const std::filesystem::path &path, const glm::uvec2 size, const uint32_t bands, const GDALDataType type) {
    gis::gdal::initialize_once();
    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName(path.empty() ? "MEM" : "GTiff");
    REQUIRE(driver != nullptr);
    auto dataset = std::make_shared<gis::Dataset>(driver->Create(path.c_str(), static_cast<int>(size.x), static_cast<int>(size.y), static_cast<int>(bands), type, nullptr));
    for (uint32_t band = 1; band <= bands; band++) {
        REQUIRE(dataset->as_gdal()->GetRasterBand(static_cast<int>(band))->Fill(10.0 * band) == CE_None);
    }
    return dataset;
}

// Each band is filled with 10 times its number. An empty path creates an in-memory raster, otherwise a GeoTIFF.
std::shared_ptr<gis::Dataset> make_raster(
    const std::filesystem::path &path, const glm::uvec2 size, const uint32_t bands, const GDALDataType type, std::array<double, 6> geo_transform, const uint32_t epsg = 3857) {
    auto dataset = make_bare_raster(path, size, bands, type);
    const gis::srs::System reference = REQUIRE_VALUE(gis::srs::from_epsg(epsg));
    REQUIRE(dataset->as_gdal()->SetSpatialRef(&reference.as_gdal()) == CE_None);
    REQUIRE(dataset->as_gdal()->SetGeoTransform(geo_transform.data()) == CE_None);
    return dataset;
}

// Georeferences `dataset` by GCPs that put its corners at the corners of `bounds` in the system with the EPSG code `epsg`.
void set_corner_gcps(gis::Dataset &dataset, const radix::tile::SrsBounds &bounds, const uint32_t epsg) {
    const glm::dvec2 size(dataset.raster_size());
    const std::array<GDAL_GCP, 4> gcps{{
        {nullptr, nullptr, 0, 0, bounds.min.x, bounds.max.y, 0},
        {nullptr, nullptr, size.x, 0, bounds.max.x, bounds.max.y, 0},
        {nullptr, nullptr, 0, size.y, bounds.min.x, bounds.min.y, 0},
        {nullptr, nullptr, size.x, size.y, bounds.max.x, bounds.min.y, 0},
    }};
    const gis::srs::System system = REQUIRE_VALUE(gis::srs::from_epsg(epsg));
    REQUIRE(dataset.as_gdal()->SetGCPs(static_cast<int>(gcps.size()), gcps.data(), &system.as_gdal()) == CE_None);
}

std::vector<float> generate(const glm::uvec2 size, const std::function<float(uint32_t, uint32_t)> &value) {
    std::vector<float> values(static_cast<size_t>(size.x) * size.y);
    for (uint32_t y = 0; y < size.y; y++) {
        for (uint32_t x = 0; x < size.x; x++) {
            values[static_cast<size_t>(y) * size.x + x] = value(x, y);
        }
    }
    return values;
}

void write(GDALRasterBand *band, std::vector<float> values) {
    const int width = band->GetXSize();
    const int height = band->GetYSize();
    REQUIRE(values.size() == static_cast<size_t>(width) * static_cast<size_t>(height));
    REQUIRE(band->RasterIO(GF_Write, 0, 0, width, height, values.data(), width, height, GDT_Float32, 0, 0) == CE_None);
}

void write_pixel(GDALRasterBand *band, const glm::uvec2 position, float value) {
    REQUIRE(band->RasterIO(GF_Write, static_cast<int>(position.x), static_cast<int>(position.y), 1, 1, &value, 1, 1, GDT_Float32, 0, 0) == CE_None);
}

class FailingBand final : public GDALRasterBand {
public:
    explicit FailingBand(GDALDataset *owner) {
        poDS = owner;
        nBand = 1;
        eDataType = GDT_Float32;
        nBlockXSize = 16;
        nBlockYSize = 1;
    }

protected:
    CPLErr IReadBlock(int, int, void *) override {
        CPLError(CE_Failure, CPLE_FileIO, "injected read failure");
        return CE_Failure;
    }
};

class FailingDataset final : public GDALDataset {
public:
    FailingDataset() : m_reference(gis::srs::webmercator().as_gdal()) {
        nRasterXSize = 16;
        nRasterYSize = 16;
        SetBand(1, new FailingBand(this));
    }

    const OGRSpatialReference *GetSpatialRef() const override {
        return &this->m_reference;
    }

    CPLErr GetGeoTransform(double *affine) override {
        const std::array<double, 6> values{0, 1, 0, 16, 0, -1};
        std::ranges::copy(values, affine);
        return CE_None;
    }

private:
    OGRSpatialReference m_reference;
};
} // namespace

TEST_CASE("read keeps the height range and matches the reference render", "[gis][DatasetReader]") {
    const std::vector at100m = {
        "/austria/at_100m_mgi.tif",
        "/austria/at_100m_epsg3857.tif",
        "/austria/at_100m_epsg4326.tif",
        "/austria/at_x149m_y100m_epsg4326.tif",
    };

    const std::vector vienna20m = {
        "/austria/vienna_20m_mgi.tif",
        "/austria/vienna_20m_epsg3857.tif",
        "/austria/vienna_20m_epsg4326.tif",
    };

    const std::vector tauern10m = {
        "/austria/tauern_10m_mgi.tif",
        "/austria/tauern_10m_epsg3857.tif",
        "/austria/tauern_10m_epsg4326.tif",
    };

    const std::vector pizbuin1m = {
        "/austria/pizbuin_1m_mgi.tif",
        "/austria/pizbuin_1m_epsg3857.tif",
        "/austria/pizbuin_1m_epsg4326.tif",
    };

    const gis::srs::System geographic_srs = gis::srs::wgs84();

    SECTION("check for min and max heights") {
        const std::array test_projections = {
            std::make_pair(gis::Projection::Geographic, 4326U),
            std::make_pair(gis::Projection::WebMercator, 3857U),
        };
        const std::array test_locations = {
        // CRS bounds, [lower limit, at least one value smaller, at least one value larger, upper limit]
#if defined(ALP_UNITTESTS_EXTENDED) && ALP_UNITTESTS_EXTENDED
            std::make_tuple(
                "at100m",
                at100m,
                radix::tile::SrsBounds{{9.5, 46.4}, {17.1, 49.0}},
                std::make_tuple(100.0f, 120.0f, 3000.0f, 3800.0f)), // Austria is between 115 and 3798m
            std::make_tuple(
                "vienna20m",
                vienna20m,
                radix::tile::SrsBounds{{16.17, 48.14}, {16.59, 48.33}},
                std::make_tuple(140.0f, 180.0f, 500.0f, 560.0f)), // vienna, between 151 and 542m
#endif
            std::make_tuple(
                "tauern10m",
                tauern10m,
                radix::tile::SrsBounds{{12.6934117, 47.0739300}, {12.6944580, 47.0748649}},
                std::make_tuple(3700.0f, 3720.0f, 3790.0f, 3800.0f)), // Grossglockner, 3798m with some surroundings
        };

        for (const auto &test : test_locations) {
            const auto [test_name, test_datasets, geographic_bounds, limits] = test;

            for (std::string dataset_name : test_datasets) {
                const auto dataset = REQUIRE_VALUE(gis::Dataset::open_shared_raster(ALP_TEST_DATA_DIR + std::string(dataset_name)));
                for (const auto &[test_projection, test_srs] : test_projections) {
                    const gis::srs::System srs = REQUIRE_VALUE(gis::srs::from_epsg(test_srs));

                    const auto srs_corners = REQUIRE_VALUE(gis::srs::transform_points(geographic_srs, srs, std::vector{geographic_bounds.min, geographic_bounds.max}));
                    const radix::tile::SrsBounds srs_bounds{srs_corners[0], srs_corners[1]};

                    require_projection_available(*dataset, srs);
                    auto reader = make_reader(dataset, test_projection, {1}, 0.0F);
                    REQUIRE(reader.projection() == test_projection);
                    if (ALP_UNITTESTS_DEBUG_IMAGES) {
                        const auto debug_samples = read_samples(reader, srs_bounds, {1000, 1000});
                        const auto s = std::string("/austria/").length();
                        const auto l = dataset_name.length() - std::string(".tif").length() - s;
                        const auto debug_file = fmt::format("./heights_{}_srs{}_{}.png", test_name, test_srs, dataset_name.substr(s, l));
                        debug_out(debug_samples.data, debug_file);
                    }

                    const auto samples = read_samples(reader, srs_bounds, {100, 111});
                    CHECK(all_valid(samples.valid));
                    const auto &heights = samples.data;
                    REQUIRE(heights.width() == 100);
                    REQUIRE(heights.height() == 111);
                    const auto [lower_bound, lower_than, higher_than, higher_bound] = limits;
                    auto [min, max] = std::ranges::minmax(heights);
                    CHECK(min > lower_bound);
                    CHECK(min < lower_than);
                    CHECK(max < higher_bound);
                    CHECK(max > higher_than);
                }
            }
        }
    }

    SECTION("compare with ref render") {
        const std::array test_data = {
#if defined(ALP_UNITTESTS_EXTENDED) && ALP_UNITTESTS_EXTENDED
            std::make_tuple(
                "at100m",
                at100m,
                radix::tile::SrsBounds{{9.5, 46.4}, {17.1, 49.0}},
                620U, 350U, 42.0, 22.0),
#endif
            std::make_tuple(
                "pizbuin1m",
                pizbuin1m,
                // Both reads use the approximate transformer; exact transforms stay below 2.7.
                radix::tile::SrsBounds{{10.105646780, 46.839864531}, {10.129815588, 46.847626067}},
                740U, 315U, 3.2, 0.006),
#if defined(ALP_UNITTESTS_EXTENDED) && ALP_UNITTESTS_EXTENDED
            std::make_tuple(
                "pizbuin1m_highres",
                pizbuin1m,
                radix::tile::SrsBounds{{10.105646780, 46.839864531}, {10.129815588, 46.847626067}},
                2000U, 850U, 10.0, 0.008),
#endif
        };

        for (const auto &test : test_data) {
            auto [test_name, datasets, ref_bounds, render_width, render_height, max_abs_diff, max_mse] = test;

            const auto ref_dataset = REQUIRE_VALUE(gis::Dataset::open_shared_raster(ALP_TEST_DATA_DIR + std::string(datasets.front())));
            require_projection_available(*ref_dataset, geographic_srs);
            auto ref_reader = make_reader(ref_dataset, gis::Projection::Geographic, {1}, 0.0F);
            const auto ref_samples = read_samples(ref_reader, ref_bounds, {render_width, render_height});
            CHECK(all_valid(ref_samples.valid));
            const auto &ref_heights = ref_samples.data;
            if (ALP_UNITTESTS_DEBUG_IMAGES) {
                debug_out(ref_heights, fmt::format("./heights_ref.png"));
            }

            for (std::string dataset_name : datasets) {
                const auto dataset = REQUIRE_VALUE(gis::Dataset::open_shared_raster(ALP_TEST_DATA_DIR + std::string(dataset_name)));
                require_projection_available(*dataset, geographic_srs);
                auto reader = make_reader(dataset, gis::Projection::Geographic, {1}, 0.0F);
                const auto samples = read_samples(reader, ref_bounds, {render_width, render_height});
                CHECK(all_valid(samples.valid));
                const auto &heights = samples.data;

                const auto s = std::string("/austria/").length();
                const auto l = dataset_name.length() - std::string(".tif").length() - s;

                if (ALP_UNITTESTS_DEBUG_IMAGES) {
                    debug_out(ref_heights, fmt::format("./heights_{}_{}.png", test_name, dataset_name.substr(s, l)));

                    auto height_diffs = radix::Raster<float>({render_width, render_height});
                    std::transform(ref_heights.begin(), ref_heights.end(), heights.begin(), height_diffs.begin(), [](auto a, auto b) { return std::abs(a - b); });
                    const auto path = fmt::format("./diffs_{}_{}.png", test_name, dataset_name.substr(s, l));
                    debug_out(height_diffs, path);
                }
                auto largest_abs_diff = 0.0;
                const auto mse = std::transform_reduce(ref_heights.begin(), ref_heights.end(), heights.begin(), 0.0, std::plus<>(), [&largest_abs_diff](auto a, auto b) {
                    const double t = std::abs(static_cast<double>(a) - static_cast<double>(b));
                    largest_abs_diff = std::max(t, largest_abs_diff);
                    return t * t;
                }) / static_cast<double>(ref_heights.buffer_length());
                CHECK(largest_abs_diff < max_abs_diff);
                CHECK(mse < max_mse);
            }
        }
    }
}

TEST_CASE("min_pixel_size is the finest source pixel size within the bounds", "[gis][DatasetReader]") {
    using Projection = gis::Projection;
    const auto open = [](const char *name) { return REQUIRE_VALUE(gis::Dataset::open_shared_raster(ALP_TEST_DATA_DIR + std::string(name))); };
    const auto reader = [](const std::shared_ptr<gis::Dataset> &dataset, const Projection projection) { return make_reader(dataset, projection, {1}, 0.0F); };
    const auto pixel_size = [](const std::shared_ptr<gis::Dataset> &dataset) {
        std::array<double, 6> geo_transform{};
        REQUIRE(dataset->as_gdal()->GetGeoTransform(geo_transform.data()) == CE_None);
        return glm::dvec2(geo_transform[1], -geo_transform[5]);
    };
    const double metres_per_degree = 6378137.0 * std::numbers::pi / 180;
    // Within all Austrian test datasets.
    const radix::tile::SrsBounds geographic_bounds{{12.0, 47.0}, {14.0, 48.0}};
    const auto mercator_corners = REQUIRE_VALUE(gis::srs::transform_points(gis::srs::wgs84(), gis::srs::webmercator(), std::vector{geographic_bounds.min, geographic_bounds.max}));
    const radix::tile::SrsBounds mercator_bounds{mercator_corners[0], mercator_corners[1]};

    SECTION("same srs as the dataset") {
        const auto mercator = open("/austria/at_100m_epsg3857.tif");
        const glm::dvec2 mercator_size = REQUIRE_VALUE(reader(mercator, Projection::WebMercator).min_pixel_size(mercator_bounds));
        CHECK(mercator_size.x == Catch::Approx(pixel_size(mercator).x));
        CHECK(mercator_size.y == Catch::Approx(pixel_size(mercator).y));

        const auto anisotropic = open("/austria/at_x149m_y100m_epsg4326.tif");
        const glm::dvec2 geographic_size = REQUIRE_VALUE(reader(anisotropic, Projection::Geographic).min_pixel_size(geographic_bounds));
        CHECK(geographic_size.x == Catch::Approx(pixel_size(anisotropic).x));
        CHECK(geographic_size.y == Catch::Approx(pixel_size(anisotropic).y));
    }

    SECTION("geographic dataset in web mercator") {
        // Web Mercator stretches y by 1 / cos(latitude), the minimum is at the southern edge.
        const auto dataset = open("/austria/at_100m_epsg4326.tif");
        const glm::dvec2 size = REQUIRE_VALUE(reader(dataset, Projection::WebMercator).min_pixel_size(mercator_bounds));
        const double south = geographic_bounds.min.y * std::numbers::pi / 180;
        CHECK(size.x == Catch::Approx(pixel_size(dataset).x * metres_per_degree).epsilon(1e-3));
        CHECK(size.y == Catch::Approx(pixel_size(dataset).y * metres_per_degree / std::cos(south)).epsilon(1e-3));
    }

    SECTION("bounds larger than the dataset are clipped to its coverage") {
        const auto dataset = open("/austria/at_100m_epsg4326.tif");
        const radix::tile::SrsBounds world{glm::dvec2(-gis::webmercator::half_extent), glm::dvec2(gis::webmercator::half_extent)};
        const glm::dvec2 size = REQUIRE_VALUE(reader(dataset, Projection::WebMercator).min_pixel_size(world));
        const radix::tile::SrsBounds dataset_bounds = REQUIRE_VALUE(dataset->bounds());
        const double south = dataset_bounds.min.y * std::numbers::pi / 180;
        CHECK(size.y == Catch::Approx(pixel_size(dataset).y * metres_per_degree / std::cos(south)).epsilon(1e-3));
    }

    SECTION("projected dataset in web mercator") {
        // About 100 m on the ground, i.e. 1 / cos(47°) * 100 m in Web Mercator, reduced slightly by the grid rotation.
        const auto dataset = open("/austria/at_100m_mgi.tif");
        const glm::dvec2 size = REQUIRE_VALUE(reader(dataset, Projection::WebMercator).min_pixel_size(mercator_bounds));
        CHECK(size.x > 130);
        CHECK(size.x < 155);
        CHECK(size.y > 130);
        CHECK(size.y < 155);
    }

    SECTION("bounds outside the coverage") {
        const auto dataset = open("/austria/at_100m_epsg4326.tif");
        const auto error = REQUIRE_ERROR(reader(dataset, Projection::Geographic).min_pixel_size({{0.0, 0.0}, {1.0, 1.0}}));
        CHECK(error.holds<gis::OutsideCoverage>());
    }
}

TEST_CASE("read wraps a geographic source wider than 360 degrees around its centre", "[gis][DatasetReader]") {
    // 0.5° pixels from -0.5 to 360.5, each storing its centre longitude. GDAL only wraps sources overlapping themselves by at most about one pixel.
    constexpr int width = 722;
    constexpr int height = 240;
    auto dataset = make_bare_raster({}, {width, height}, 1, GDT_Float32);
    std::array<double, 6> geo_transform{-0.5, 0.5, 0, 60, 0, -0.5};
    REQUIRE(dataset->as_gdal()->SetGeoTransform(geo_transform.data()) == CE_None);
    const gis::srs::System reference = gis::srs::wgs84();
    REQUIRE(dataset->as_gdal()->SetSpatialRef(&reference.as_gdal()) == CE_None);
    std::vector<float> longitudes(static_cast<size_t>(width) * height);
    for (size_t i = 0; i < longitudes.size(); i++) {
        longitudes[i] = static_cast<float>(-0.25 + 0.5 * static_cast<double>(i % width));
    }
    REQUIRE(dataset->as_gdal()->GetRasterBand(1)->RasterIO(GF_Write, 0, 0, width, height, longitudes.data(), width, height, GDT_Float32, 0, 0) == CE_None);

    auto reader = make_reader(dataset, gis::Projection::WebMercator, {1}, 0.0F);
    const double degree = gis::webmercator::half_extent / 180;
    for (const double longitude : {-90.0, 90.0}) {
        const auto samples = read_samples(reader, {{(longitude - 1) * degree, -degree}, {(longitude + 1) * degree, degree}}, {4, 4});
        CHECK(all_valid(samples.valid));
        for (const float value : samples.data.buffer()) {
            CHECK(value == Catch::Approx(longitude < 0 ? longitude + 360 : longitude).margin(1.5));
        }
    }
}

TEST_CASE("read reads the selected bands in order", "[gis][DatasetReader]") {
    const auto bounds = gis::webmercator::compute_tile_bounds({5, {16, 15}});
    const auto geo_transform = geo_transform_for(bounds, glm::uvec2(32));
    SECTION("scalar") {
        auto reader = make_reader(make_raster({}, glm::uvec2(32), 3, GDT_Float32, geo_transform), gis::Projection::WebMercator, {2}, 0.0F);
        for (const auto size : {glm::uvec2(32), glm::uvec2(24, 40)}) {
            const auto samples = read_samples(reader, bounds, size);
            CHECK(all_valid(samples.valid));
            CHECK(std::ranges::all_of(samples.data, [](const float value) { return value == Catch::Approx(20); }));
        }
    }
    SECTION("RGB") {
        auto reader = make_reader(make_raster({}, glm::uvec2(32), 4, GDT_Byte, geo_transform), gis::Projection::WebMercator, {3, 1, 4}, glm::u8vec3(0));
        for (const auto size : {glm::uvec2(32), glm::uvec2(24, 40)}) {
            const auto samples = read_samples(reader, bounds, size);
            CHECK(all_valid(samples.valid));
            CHECK(std::ranges::all_of(samples.data, [](const glm::u8vec3 value) { return value == glm::u8vec3(30, 10, 40); }));
        }
    }
}

TEST_CASE("make fails as unsupported band type for a band of another type", "[gis][DatasetReader]") {
    const auto bounds = gis::webmercator::compute_tile_bounds({5, {16, 15}});
    const auto raster = [&](const GDALDataType type) { return make_raster({}, glm::uvec2(16), 3, type, geo_transform_for(bounds, glm::uvec2(16))); };
    SECTION("scalar") {
        const GDALDataType type = GENERATE(GDT_Byte, GDT_Int16, GDT_Float64);

        const auto error = REQUIRE_ERROR(gis::DatasetReader<float>::make(raster(type), gis::Projection::WebMercator, {1}, 0.0F));

        CHECK(error.holds_code(gis::UnsupportedBandType{.band = 1, .type = type, .required = GDT_Float32}));
    }
    SECTION("RGB") {
        const GDALDataType type = GENERATE(GDT_Float32, GDT_UInt16);

        const auto error = REQUIRE_ERROR(gis::DatasetReader<glm::u8vec3>::make(raster(type), gis::Projection::WebMercator, {1, 2, 3}, glm::u8vec3(0)));

        CHECK(error.holds_code(gis::UnsupportedBandType{.band = 1, .type = type, .required = GDT_Byte}));
    }
}

TEST_CASE("make fails as missing band for a band number the dataset does not have", "[gis][DatasetReader]") {
    const auto bounds = gis::webmercator::compute_tile_bounds({5, {16, 15}});
    const auto raster = [&](const GDALDataType type) { return make_raster({}, glm::uvec2(16), 3, type, geo_transform_for(bounds, glm::uvec2(16))); };
    const uint32_t band = GENERATE(0U, 4U);
    SECTION("scalar") {
        const auto error = REQUIRE_ERROR(gis::DatasetReader<float>::make(raster(GDT_Float32), gis::Projection::WebMercator, {band}, 0.0F));

        CHECK(error.holds_code(gis::MissingBand{.band = band, .band_count = 3}));
    }
    SECTION("RGB") {
        const auto error = REQUIRE_ERROR(gis::DatasetReader<glm::u8vec3>::make(raster(GDT_Byte), gis::Projection::WebMercator, {1, band, 3}, glm::u8vec3(0)));

        CHECK(error.holds_code(gis::MissingBand{.band = band, .band_count = 3}));
    }
}

TEST_CASE("make rejects a default pixel that is not finite", "[gis][DatasetReader]") {
    const auto bounds = gis::webmercator::compute_tile_bounds({5, {16, 15}});
    const auto dataset = make_raster({}, glm::uvec2(16), 1, GDT_Float32, geo_transform_for(bounds, glm::uvec2(16)));
    const float default_pixel = GENERATE(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity());

    REQUIRE_ASSERTS(gis::DatasetReader<float>::make(dataset, gis::Projection::WebMercator, {1}, default_pixel));
}

TEST_CASE("make fails as missing srs for a raster without one", "[gis][DatasetReader]") {
    const auto bounds = gis::webmercator::compute_tile_bounds({5, {16, 15}});
    const auto dataset = make_raster({}, glm::uvec2(16), 1, GDT_Float32, geo_transform_for(bounds, glm::uvec2(16)));
    REQUIRE(dataset->as_gdal()->SetSpatialRef(nullptr) == CE_None);

    const auto error = REQUIRE_ERROR(gis::DatasetReader<float>::make(dataset, gis::Projection::WebMercator, {1}, 0.0F));

    CHECK(error.holds<gis::MissingSrs>());
}

TEST_CASE("make fails as missing georeferencing for a raster without any", "[gis][DatasetReader]") {
    const auto dataset = make_bare_raster({}, glm::uvec2(16), 1, GDT_Float32);

    const auto error = REQUIRE_ERROR(gis::DatasetReader<float>::make(dataset, gis::Projection::WebMercator, {1}, 0.0F));

    CHECK(error.holds<gis::MissingGeoreferencing>());
}

TEST_CASE("make fails as ambiguous for a raster with two kinds of georeferencing", "[gis][DatasetReader]") {
    const radix::tile::SrsBounds bounds{{0, 0}, {1000000, 1000000}};
    std::shared_ptr<gis::Dataset> dataset;
    SECTION("geotransform and GCPs") {
        dataset = make_raster({}, glm::uvec2(16), 1, GDT_Float32, geo_transform_for(bounds, glm::uvec2(16)));
        set_corner_gcps(*dataset, bounds, 3857);
    }
    SECTION("GCPs and RPCs") {
        dataset = make_bare_raster({}, glm::uvec2(16), 1, GDT_Float32);
        set_corner_gcps(*dataset, bounds, 3857);
        REQUIRE(dataset->as_gdal()->SetMetadataItem("LINE_OFF", "1", "RPC") == CE_None);
    }

    const auto error = REQUIRE_ERROR(gis::DatasetReader<float>::make(dataset, gis::Projection::WebMercator, {1}, 0.0F));

    CHECK(error.holds<gis::AmbiguousGeoreferencing>());
}

TEST_CASE("make fails as missing georeferencing for a reprojected geographic source without a geotransform", "[gis][DatasetReader]") {
    const auto dataset = make_bare_raster({}, glm::uvec2(16), 1, GDT_Float32);
    set_corner_gcps(*dataset, {{10, 46}, {11, 47}}, 4326);

    const auto error = REQUIRE_ERROR(gis::DatasetReader<float>::make(dataset, gis::Projection::WebMercator, {1}, 0.0F));

    CHECK(error.holds<gis::MissingGeoreferencing>());
}

TEST_CASE("make fails as unsupported reprojection for a system GDAL cannot transform", "[gis][DatasetReader]") {
    const auto dataset = make_bare_raster({}, glm::uvec2(16), 1, GDT_Float32);
    std::array<double, 6> geo_transform{0, 1, 0, 16, 0, -1};
    REQUIRE(dataset->as_gdal()->SetGeoTransform(geo_transform.data()) == CE_None);
    OGRSpatialReference local;
    REQUIRE(local.SetLocalCS("arbitrary") == OGRERR_NONE);
    REQUIRE(dataset->as_gdal()->SetSpatialRef(&local) == CE_None);

    const auto error = REQUIRE_ERROR(gis::DatasetReader<float>::make(dataset, gis::Projection::WebMercator, {1}, 0.0F));

    CHECK(error.holds<gis::UnsupportedReprojection>());
}

TEST_CASE("read rejects an empty or oversized size and bounds that are not finite and nonempty", "[gis][DatasetReader]") {
    const auto bounds = gis::webmercator::compute_tile_bounds({5, {16, 15}});
    const auto raster = [&](const GDALDataType type) { return make_raster({}, glm::uvec2(16), 3, type, geo_transform_for(bounds, glm::uvec2(16))); };
    SECTION("scalar") {
        auto reader = make_reader(raster(GDT_Float32), gis::Projection::WebMercator, {1}, 0.0F);
        const uint32_t beyond_int = static_cast<uint32_t>(INT_MAX) + 1;
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double infinity = std::numeric_limits<double>::infinity();
        const auto [request_bounds, size] = GENERATE_COPY(table<radix::tile::SrsBounds, glm::uvec2>({
            {bounds, {0, 16}},
            {bounds, {16, 0}},
            {bounds, {beyond_int, 1}},
            {bounds, {1, beyond_int}},
            {bounds, {INT_MAX, INT_MAX}},
            {{bounds.min, bounds.min}, {16, 16}},
            {{bounds.max, bounds.min}, {16, 16}},
            {{{nan, 0}, {1, 1}}, {16, 16}},
            {{{-infinity, 0}, {1, 1}}, {16, 16}},
            {{{-DBL_MAX, 0}, {DBL_MAX, 1}}, {16, 16}},
        }));

        REQUIRE_ASSERTS(reader.read(request_bounds, size));
    }
    SECTION("RGB") {
        auto reader = make_reader(raster(GDT_Byte), gis::Projection::WebMercator, {1, 2, 3}, glm::u8vec3(0));

        REQUIRE_ASSERTS(reader.read(bounds, {INT_MAX, INT_MAX}));
    }
}

TEST_CASE("read fails as a window read error when GDAL fails to read", "[gis][DatasetReader]") {
    gis::gdal::initialize_once();
    auto reader = make_reader(std::make_shared<gis::Dataset>(new FailingDataset()), gis::Projection::WebMercator, {1}, 0.0F);

    const auto error = REQUIRE_ERROR(reader.read({{0, 0}, {16, 16}}, {8, 8}));

    CHECK(error.holds<gis::WindowReadError>());
}

TEST_CASE("read maps a GCP source through the system of its GCPs", "[gis][DatasetReader]") {
    // About 0 to 9 degrees east and north.
    const auto dataset = make_bare_raster({}, glm::uvec2(32), 1, GDT_Float32);
    set_corner_gcps(*dataset, {{0, 0}, {1000000, 1000000}}, 3857);
    auto reader = make_reader(dataset, gis::Projection::Geographic, {1}, 0.0F);

    const auto samples = read_samples(reader, {{2, 2}, {6, 6}}, glm::uvec2(16));

    CHECK(all_valid(samples.valid));
    CHECK(std::ranges::all_of(samples.data, [](const float value) { return value == Catch::Approx(10); }));
}

TEST_CASE("read combines the effective source masks", "[gis][DatasetReader]") {
    const auto bounds = gis::webmercator::compute_tile_bounds({5, {16, 15}});
    const glm::uvec2 size(32);
    const glm::u8vec3 fallback(1, 2, 3);
    auto dataset = make_raster({}, size, 4, GDT_Byte, geo_transform_for(bounds, size));
    GDALDataset *gdal_dataset = dataset->as_gdal();
    using Region = std::function<bool(uint32_t, uint32_t)>;
    const Region left = [](const uint32_t x, uint32_t) { return x < 16; };
    const Region top = [](uint32_t, const uint32_t y) { return y < 16; };
    const auto values = [&](const Region &valid) { return generate(size, [&](const uint32_t x, const uint32_t y) { return valid(x, y) ? 255.f : 0.f; }); };
    const auto mask = [&](const int band, const int flags, const Region &valid) {
        REQUIRE(gdal_dataset->GetRasterBand(band)->CreateMaskBand(flags) == CE_None);
        write(gdal_dataset->GetRasterBand(band)->GetMaskBand(), values(valid));
    };
    Region expected = [](uint32_t, uint32_t) { return true; };
    SECTION("all valid") {}
    SECTION("band NoData") {
        REQUIRE(gdal_dataset->GetRasterBand(2)->SetNoDataValue(20) == CE_None);
        expected = [](uint32_t, uint32_t) { return false; };
    }
    SECTION("explicit mask of one band") {
        mask(2, 0, left);
        expected = left;
    }
    SECTION("shared per-dataset mask") {
        mask(1, GMF_PER_DATASET, top);
        REQUIRE((gdal_dataset->GetRasterBand(3)->GetMaskFlags() & GMF_PER_DATASET) != 0);
        expected = top;
    }
    SECTION("alpha band") {
        REQUIRE(gdal_dataset->GetRasterBand(4)->SetColorInterpretation(GCI_AlphaBand) == CE_None);
        write(gdal_dataset->GetRasterBand(4), values(top));
        REQUIRE((gdal_dataset->GetRasterBand(1)->GetMaskFlags() & GMF_ALPHA) != 0);
        expected = top;
    }
    SECTION("different masks per band") {
        mask(1, 0, left);
        mask(3, 0, top);
        expected = [&](const uint32_t x, const uint32_t y) { return left(x, y) && top(x, y); };
    }
    auto reader = make_reader(dataset, gis::Projection::WebMercator, {1, 2, 3}, fallback);
    const auto samples = read_samples(reader, bounds, size);
    for (uint32_t y = 0; y < size.y; y++) {
        for (uint32_t x = 0; x < size.x; x++) {
            INFO(fmt::format("pixel {}, {}", x, y));
            CHECK((samples.valid.pixel({x, y}) != 0) == expected(x, y));
            CHECK(samples.data.pixel({x, y}) == (expected(x, y) ? glm::u8vec3(10, 20, 30) : fallback));
        }
    }
}

TEST_CASE("read applies the mask of a single band", "[gis][DatasetReader]") {
    const auto bounds = gis::webmercator::compute_tile_bounds({5, {16, 15}});
    const glm::uvec2 size(32);
    auto dataset = make_raster({}, size, 1, GDT_Float32, geo_transform_for(bounds, size));
    GDALRasterBand *band = dataset->as_gdal()->GetRasterBand(1);
    const auto right = [](const uint32_t x, uint32_t) { return x >= 16; };
    SECTION("NoData") {
        write(band, generate(size, [&](const uint32_t x, const uint32_t y) { return right(x, y) ? 10.f : 0.f; }));
        REQUIRE(band->SetNoDataValue(0) == CE_None);
    }
    SECTION("explicit mask") {
        REQUIRE(band->CreateMaskBand(0) == CE_None);
        write(band->GetMaskBand(), generate(size, [&](const uint32_t x, const uint32_t y) { return right(x, y) ? 255.f : 0.f; }));
    }
    auto reader = make_reader(dataset, gis::Projection::WebMercator, {1}, -1.0F);
    const auto samples = read_samples(reader, bounds, size);
    for (uint32_t y = 0; y < size.y; y++) {
        for (uint32_t x = 0; x < size.x; x++) {
            INFO(fmt::format("pixel {}, {}", x, y));
            REQUIRE((samples.valid.pixel({x, y}) != 0) == right(x, y));
            REQUIRE(samples.data.pixel({x, y}) == (right(x, y) ? 10.f : -1.f));
        }
    }
}

TEST_CASE("read prefers an explicit mask over a conflicting NoData value", "[gis][DatasetReader]") {
    const auto bounds = gis::webmercator::compute_tile_bounds({5, {16, 15}});
    auto dataset = make_raster({}, glm::uvec2(32), 1, GDT_Float32, geo_transform_for(bounds, glm::uvec2(32)));
    GDALRasterBand *band = dataset->as_gdal()->GetRasterBand(1);
    REQUIRE(band->SetNoDataValue(10) == CE_None);
    REQUIRE(band->CreateMaskBand(0) == CE_None);
    REQUIRE(band->GetMaskBand()->Fill(255) == CE_None);
    auto reader = make_reader(dataset, gis::Projection::WebMercator, {1}, 0.0F);
    const auto samples = read_samples(reader, bounds, glm::uvec2(24));
    CHECK(all_valid(samples.valid));
    CHECK(std::ranges::all_of(samples.data, [](const float value) { return value == Catch::Approx(10); }));
}

TEST_CASE("read keeps valid pixels equal to sentinels or the default", "[gis][DatasetReader]") {
    const auto bounds = gis::webmercator::compute_tile_bounds({5, {16, 15}});
    const auto geo_transform = geo_transform_for(bounds, glm::uvec2(32));
    SECTION("black RGB with a black default") {
        auto dataset = make_raster({}, glm::uvec2(32), 3, GDT_Byte, geo_transform);
        for (int band = 1; band <= 3; band++) {
            REQUIRE(dataset->as_gdal()->GetRasterBand(band)->Fill(0) == CE_None);
        }
        auto reader = make_reader(dataset, gis::Projection::WebMercator, {1, 2, 3}, glm::u8vec3(0));
        const auto samples = read_samples(reader, bounds, glm::uvec2(32));
        CHECK(all_valid(samples.valid));
        CHECK(std::ranges::all_of(samples.data, [](const glm::u8vec3 value) { return value == glm::u8vec3(0); }));
    }
    SECTION("scalar equal to the default") {
        auto reader = make_reader(make_raster({}, glm::uvec2(32), 1, GDT_Float32, geo_transform), gis::Projection::WebMercator, {1}, 10.0F);
        const auto samples = read_samples(reader, bounds, glm::uvec2(32));
        CHECK(all_valid(samples.valid));
        CHECK(std::ranges::all_of(samples.data, [](const float value) { return value == 10; }));
    }
    SECTION("scalar -32768 without NoData") {
        auto dataset = make_raster({}, glm::uvec2(32), 1, GDT_Float32, geo_transform);
        REQUIRE(dataset->as_gdal()->GetRasterBand(1)->Fill(-32768) == CE_None);
        auto reader = make_reader(dataset, gis::Projection::WebMercator, {1}, 0.0F);
        const auto samples = read_samples(reader, bounds, glm::uvec2(32));
        CHECK(all_valid(samples.valid));
        CHECK(std::ranges::all_of(samples.data, [](const float value) { return value == -32768; }));
    }
}

namespace {
template <typename Pixel>
void check_coverage(const GDALDataType type, const std::array<uint32_t, gis::DatasetReader<Pixel>::channel_count> bands, const Pixel inside, const Pixel fallback) {
    const auto bounds = gis::webmercator::compute_tile_bounds({5, {16, 15}});
    const glm::uvec2 size(32);
    // The source covers the western half of the output.
    auto dataset = make_raster({}, {16, 32}, static_cast<uint32_t>(bands.size()), type, geo_transform_for(bounds, size));
    auto reader = make_reader(dataset, gis::Projection::WebMercator, bands, fallback);
    const auto partial = read_samples(reader, bounds, size);
    for (uint32_t y = 0; y < size.y; y++) {
        for (uint32_t x = 0; x < size.x; x++) {
            INFO(fmt::format("pixel {}, {}", x, y));
            CHECK((partial.valid.pixel({x, y}) != 0) == (x < 16));
            CHECK(partial.data.pixel({x, y}) == (x < 16 ? inside : fallback));
        }
    }
    const auto absent = read_samples(reader, gis::webmercator::compute_tile_bounds({5, {2, 2}}), size);
    CHECK(std::ranges::none_of(absent.valid, [](const uint8_t value) { return value != 0; }));
    CHECK(std::ranges::all_of(absent.data, [&](const Pixel value) { return value == fallback; }));
}
} // namespace

TEST_CASE("read returns defaults outside the source coverage", "[gis][DatasetReader]") {
    SECTION("scalar") {
        check_coverage<float>(GDT_Float32, {1}, 10.0F, 7.5F);
    }
    SECTION("RGB") {
        check_coverage<glm::u8vec3>(GDT_Byte, {1, 2, 3}, glm::u8vec3(10, 20, 30), glm::u8vec3(1, 2, 3));
    }
}

TEST_CASE("read replaces nonfinite float results with the default", "[gis][DatasetReader]") {
    const float nonfinite = GENERATE(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());
    const auto bounds = gis::webmercator::compute_tile_bounds({5, {16, 15}});
    auto dataset = make_raster({}, glm::uvec2(32), 1, GDT_Float32, geo_transform_for(bounds, glm::uvec2(32)));
    GDALRasterBand *band = dataset->as_gdal()->GetRasterBand(1);
    auto reader = make_reader(dataset, gis::Projection::WebMercator, {1}, 7.5F);
    SECTION("everywhere") {
        REQUIRE(band->Fill(nonfinite) == CE_None);
        for (const auto size : {glm::uvec2(32), glm::uvec2(24)}) {
            const auto samples = read_samples(reader, bounds, size);
            CHECK(std::ranges::none_of(samples.valid, [](const uint8_t value) { return value != 0; }));
            CHECK(std::ranges::all_of(samples.data, [](const float value) { return value == 7.5F; }));
        }
    }
    SECTION("one unmasked sample contaminates its filter support") {
        write_pixel(band, {16, 16}, nonfinite);
        const auto samples = read_samples(reader, bounds, glm::uvec2(24));
        for (const auto [value, validity] : std::views::zip(samples.data.buffer(), samples.valid.buffer())) {
            if (validity != 0) {
                CHECK(value == Catch::Approx(10));
            } else {
                CHECK(value == 7.5F);
            }
        }
        // Centred on the nonfinite sample, and far from it.
        CHECK(samples.valid.pixel({12, 12}) == 0);
        CHECK(samples.valid.pixel({0, 0}) != 0);
    }
}

TEST_CASE("read excludes a sample invalid in one channel from all channels", "[gis][DatasetReader]") {
    const auto bounds = gis::webmercator::compute_tile_bounds({5, {16, 15}});
    auto dataset = make_raster({}, glm::uvec2(32), 3, GDT_Byte, geo_transform_for(bounds, glm::uvec2(32)));
    GDALDataset *gdal_dataset = dataset->as_gdal();
    for (int band = 1; band <= 3; band++) {
        REQUIRE(gdal_dataset->GetRasterBand(band)->Fill(100) == CE_None);
    }
    // A red outlier, invalid only through the green NoData.
    write_pixel(gdal_dataset->GetRasterBand(1), {16, 16}, 250);
    write_pixel(gdal_dataset->GetRasterBand(2), {16, 16}, 0);
    REQUIRE(gdal_dataset->GetRasterBand(2)->SetNoDataValue(0) == CE_None);
    const glm::u8vec3 fallback(1, 2, 3);
    auto reader = make_reader(dataset, gis::Projection::WebMercator, {1, 2, 3}, fallback);
    // GDAL does not write outputs centred on an invalid sample. The Lanczos supports of the others contain valid samples. Warping each channel with its own mask, red would include the outlier around it.
    const auto samples = read_samples(reader, bounds, glm::uvec2(24));
    for (uint32_t y = 0; y < 24; y++) {
        for (uint32_t x = 0; x < 24; x++) {
            INFO(fmt::format("pixel {}, {}", x, y));
            const bool on_outlier = x == 12 && y == 12;
            CHECK((samples.valid.pixel({x, y}) != 0) == !on_outlier);
            CHECK(samples.data.pixel({x, y}) == (on_outlier ? fallback : glm::u8vec3(100)));
        }
    }
}

TEST_CASE("read reprojects within the approximation tolerance", "[gis][DatasetReader]") {
    // 0.1° pixels storing the longitude and latitude of their centres.
    constexpr uint32_t side = 200;
    auto dataset = make_raster({}, glm::uvec2(side), 2, GDT_Float32, {0, 0.1, 0, 20, 0, -0.1}, 4326);
    write(dataset->as_gdal()->GetRasterBand(1), generate(glm::uvec2(side), [](const uint32_t x, uint32_t) { return static_cast<float>((x + 0.5) * 0.1); }));
    write(dataset->as_gdal()->GetRasterBand(2), generate(glm::uvec2(side), [](uint32_t, const uint32_t y) { return static_cast<float>(20 - (y + 0.5) * 0.1); }));
    const auto corners = REQUIRE_VALUE(gis::srs::transform_points(gis::srs::wgs84(), gis::srs::webmercator(), std::vector{glm::dvec2(5, 5), glm::dvec2(15, 15)}));
    const radix::tile::SrsBounds bounds{corners[0], corners[1]};
    const glm::uvec2 size(64, 80);
    const double radius = 6378137;
    for (const uint32_t band : {1U, 2U}) {
        auto reader = make_reader(dataset, gis::Projection::WebMercator, {band}, 0.0F);
        const auto samples = read_samples(reader, bounds, size);
        CHECK(all_valid(samples.valid));
        for (uint32_t y = 0; y < size.y; y++) {
            for (uint32_t x = 0; x < size.x; x++) {
                const double mercator_x = bounds.min.x + (x + 0.5) * bounds.width() / size.x;
                const double mercator_y = bounds.max.y - (y + 0.5) * bounds.height() / size.y;
                const double expected = band == 1 ? mercator_x / radius * 180 / std::numbers::pi
                                                  : (2 * std::atan(std::exp(mercator_y / radius)) - std::numbers::pi / 2) * 180 / std::numbers::pi;
                // Within 0.2 source pixels.
                CHECK(samples.data.pixel({x, y}) == Catch::Approx(expected).margin(0.02));
            }
        }
    }
}

TEST_CASE("read reads a VRT mosaic like a single raster at base resolution", "[gis][DatasetReader]") {
    TempDirectory directory("dataset-reader");
    {
        make_raster(directory.path() / "left.tif", glm::uvec2(16), 1, GDT_Float32, {0, 1, 0, 16, 0, -1});
        auto right = make_raster(directory.path() / "right.tif", glm::uvec2(16), 1, GDT_Float32, {16, 1, 0, 16, 0, -1});
        REQUIRE(right->as_gdal()->GetRasterBand(1)->Fill(50) == CE_None);
    }
    const auto vrt_path = directory.path() / "mosaic.vrt";
    {
        std::ofstream vrt(vrt_path);
        vrt << fmt::format(R"(<VRTDataset rasterXSize="32" rasterYSize="16">
<SRS>EPSG:3857</SRS><GeoTransform>0,1,0,16,0,-1</GeoTransform>
<VRTRasterBand dataType="Float32" band="1">
<SimpleSource><SourceFilename>{}</SourceFilename><SourceBand>1</SourceBand><SrcRect xOff="0" yOff="0" xSize="16" ySize="16"/><DstRect xOff="0" yOff="0" xSize="16" ySize="16"/></SimpleSource>
<SimpleSource><SourceFilename>{}</SourceFilename><SourceBand>1</SourceBand><SrcRect xOff="0" yOff="0" xSize="16" ySize="16"/><DstRect xOff="16" yOff="0" xSize="16" ySize="16"/></SimpleSource>
</VRTRasterBand></VRTDataset>)",
            (directory.path() / "left.tif").string(),
            (directory.path() / "right.tif").string());
        REQUIRE(vrt.good());
    }
    auto mosaic = make_reader(REQUIRE_VALUE(gis::Dataset::open_shared_raster(vrt_path)), gis::Projection::WebMercator, {1}, 0.0F);
    const radix::tile::SrsBounds window{{8.25, 0.25}, {24.25, 16.25}};
    const auto combined = read_samples(mosaic, window, glm::uvec2(16));
    auto reference = make_raster({}, glm::uvec2(32), 1, GDT_Float32, {0, 1, 0, 16, 0, -1});
    write(reference->as_gdal()->GetRasterBand(1), generate(glm::uvec2(32), [](const uint32_t x, uint32_t) { return x < 16 ? 10.f : 50.f; }));
    auto single = make_reader(reference, gis::Projection::WebMercator, {1}, 0.0F);
    const auto expected = read_samples(single, window, glm::uvec2(16));
    // Compare the source assembly, not GDAL's filter formula.
    for (uint32_t x = 0; x < 16; x++) {
        CHECK(combined.valid.pixel({x, 8}) != 0);
        CHECK(combined.data.pixel({x, 8}) == Catch::Approx(expected.data.pixel({x, 8})));
    }
    const int level = 2;
    REQUIRE(reference->as_gdal()->BuildOverviews("NEAREST", 1, &level, 0, nullptr, nullptr, nullptr) == CE_None);
    REQUIRE(reference->as_gdal()->GetRasterBand(1)->GetOverview(0)->Fill(222) == CE_None);
    const auto base = read_samples(single, {{0, -16}, {32, 16}}, glm::uvec2(16));
    CHECK(base.valid.pixel({4, 8}) != 0);
    CHECK(base.data.pixel({4, 8}) != 222);
}

TEST_CASE("read agrees across adjacent output windows", "[gis][DatasetReader]") {
    // 0.25° pixels storing the sum of their column and row, away from the antimeridian.
    auto dataset = make_raster({}, glm::uvec2(64), 1, GDT_Float32, {0, 0.25, 0, 8, 0, -0.25}, 4326);
    write(dataset->as_gdal()->GetRasterBand(1), generate(glm::uvec2(64), [](const uint32_t x, const uint32_t y) { return static_cast<float>(x + y); }));
    auto reader = make_reader(dataset, gis::Projection::WebMercator, {1}, 0.0F);
    const double width = 2 * gis::webmercator::half_extent / 180;
    const glm::dvec2 centre(4 * width, 0);
    const auto west = read_samples(reader, {centre - glm::dvec2(width, width / 2), centre + glm::dvec2(0, width / 2)}, glm::uvec2(16));
    const auto east = read_samples(reader, {centre - glm::dvec2(0, width / 2), centre + glm::dvec2(width, width / 2)}, glm::uvec2(16));
    const auto joined = read_samples(reader, {centre - glm::dvec2(width), centre + glm::dvec2(width)}, glm::uvec2(32));
    CHECK(all_valid(west.valid));
    CHECK(all_valid(east.valid));
    CHECK(all_valid(joined.valid));
    // Each window may deviate by the approximation error of 0.125 source pixels per axis.
    for (uint32_t row = 0; row < 16; row++) {
        for (uint32_t column = 0; column < 16; column++) {
            CHECK(west.data.pixel({column, row}) == Catch::Approx(joined.data.pixel({column, row + 8})).margin(0.5));
            CHECK(east.data.pixel({column, row}) == Catch::Approx(joined.data.pixel({column + 16, row + 8})).margin(0.5));
        }
    }
}

TEST_CASE("read ignores the GDAL threading environment", "[gis][DatasetReader]") {
    struct Configuration {
        const char *name;
        std::optional<std::string> previous = std::nullopt;

        Configuration(const char *option, const char *value) : name(option) {
            if (const char *old = CPLGetConfigOption(name, nullptr)) {
                previous = old;
            }
            CPLSetConfigOption(name, value);
        }

        ~Configuration() {
            CPLSetConfigOption(name, previous ? previous->c_str() : nullptr);
        }
    };
    const Configuration threads("GDAL_NUM_THREADS", "2");
    const Configuration chunks("WARP_THREAD_CHUNK_SIZE", "1");
    const auto bounds = gis::webmercator::compute_tile_bounds({5, {16, 15}});
    auto reader = make_reader(make_raster({}, glm::uvec2(32), 1, GDT_Float32, geo_transform_for(bounds, glm::uvec2(32))), gis::Projection::WebMercator, {1}, 0.0F);
    const auto samples = read_samples(reader, bounds, glm::uvec2(24));
    CHECK(CPLGetLastErrorType() < CE_Failure);
    CHECK(all_valid(samples.valid));
}

TEST_CASE("read finds a coarse projected source in a world tile", "[gis][DatasetReader]") {
    // Many probes of the world tile fail to transform into UTM.
    auto dataset = make_raster({}, glm::uvec2(4), 1, GDT_Float32, {-3500000, 2000000, 0, 4000000, 0, -2000000}, 32632);
    auto reader = make_reader(dataset, gis::Projection::WebMercator, {1}, 0.0F);
    const auto samples = read_samples(reader, gis::webmercator::compute_tile_bounds({0, {0, 0}}), glm::uvec2(16));
    // Around 11° east and north, inside the footprint.
    CHECK(samples.valid.pixel({8, 7}) != 0);
    CHECK(samples.data.pixel({8, 7}) == Catch::Approx(10));
}

TEST_CASE("read finds a source smaller than GDAL's probe spacing", "[gis][DatasetReader]") {
    auto reader = make_reader(make_raster({}, glm::uvec2(32), 1, GDT_Float32, {100, 1, 0, 200, 0, -1}), gis::Projection::WebMercator, {1}, 0.0F);
    const auto samples = read_samples(reader, {{0, 0}, {4096, 4096}}, glm::uvec2(4096));
    CHECK(samples.valid.pixel({116, 4096 - 184}) != 0);
    CHECK(samples.data.pixel({116, 4096 - 184}) == Catch::Approx(10));
}
