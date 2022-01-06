#include <array>
#include <bits/ranges_algo.h>
#include <catch2/catch.hpp>
#include <chrono>
#include <fmt/core.h>
#include <iostream>
#include <numeric>
#include <ogr_spatialref.h>
#include <string>
#include <tuple>
#include <utility>
#include <algorithm>

#include "Dataset.h"
#include "DatasetReader.h"
#include "Image.h"
#include "Tiler.h"
#include "ctb/GlobalMercator.hpp"
#include "ctb/GlobalGeodetic.hpp"
#include "ctb/types.hpp"
#include "util.h"

void checkBounds(const ctb::CRSBounds& a, const ctb::CRSBounds& b) {
  REQUIRE(a.getHeight() > 0);
  REQUIRE(a.getWidth() > 0);
  REQUIRE(b.getHeight() > 0);
  REQUIRE(b.getWidth() > 0);

  const auto heightErrorIn = std::abs(a.getHeight() - b.getHeight()) / a.getHeight();
  const auto widthErrorIn = std::abs(a.getWidth() - b.getWidth()) / a.getWidth();
//  fmt::print("height error = {}, width error = {}\n", heightErrorIn, widthErrorIn);
  REQUIRE(heightErrorIn < 0.001);
  REQUIRE(widthErrorIn < 0.001);
}

TEST_CASE("datasets are as expected") {
  auto d_mgi = Dataset(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");
  auto d_wgs84 = Dataset(ATB_TEST_DATA_DIR "/austria/at_wgs84.tif");

  OGRSpatialReference webmercator;
  webmercator.importFromEPSG(3857);
  webmercator.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  SECTION("SRS") {
    REQUIRE_FALSE(d_mgi.srs().IsEmpty());
    REQUIRE_FALSE(d_wgs84.srs().IsEmpty());
    const auto wgs84 = d_wgs84.srs();
    REQUIRE_FALSE(d_mgi.srs().IsSame(&wgs84));
  }

  SECTION("bounds") {
    {
//      fmt::print("webmercator: \n");
      checkBounds(d_mgi.bounds(webmercator), d_wgs84.bounds(webmercator));
    }
    {
//      fmt::print("d_wgs84: \n");
      checkBounds(d_mgi.bounds(d_wgs84.srs()), d_wgs84.bounds(d_wgs84.srs()));
    }
//    {
//      // doesn't work: wgs84 was created by projecting the original mgi data. since wgs84 is warped, the projection creates a border.
//      //               when backprojecting to mgi, there is still a border in the raster data (but widthout height information).
//      //               this is not an error in the code, and impossible to avoid when rerasterising warped raster data
//      fmt::print("d_mgi: \n");
//      checkBounds(d_mgi.bounds(d_mgi.srs()), d_wgs84.bounds(d_mgi.srs()));
//    }
  }
  SECTION("resolution") {
    REQUIRE(d_mgi.widthInPixels() == 620);
    REQUIRE(d_mgi.heightInPixels() == 350);
    REQUIRE(d_mgi.n_bands() == 1);
    REQUIRE(std::abs(d_mgi.pixelWidthIn(webmercator) - 573000.0/620) / 573000.0/620 < 0.01);
    REQUIRE(std::abs(d_mgi.pixelHeightIn(webmercator) - 294000.0/350) / 294000.0/350 < 0.01);
    const auto grid = ctb::GlobalMercator();
    const auto gridResuloution = std::min(d_mgi.pixelWidthIn(grid.getSRS()), d_mgi.pixelHeightIn(grid.getSRS()));
    REQUIRE(grid.zoomForResolution(gridResuloution) == 7);
  }
}

TEST_CASE("bbox width pixels") {
  auto d_mgi = Dataset(ATB_TEST_DATA_DIR "/austria/at_mgi.tif");
  auto d_wgs84 = Dataset(ATB_TEST_DATA_DIR "/austria/at_wgs84.tif");

  OGRSpatialReference webmercator;
  webmercator.importFromEPSG(3857);
  webmercator.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  REQUIRE(d_mgi.widthInPixels(d_mgi.bounds(), d_mgi.srs()) == Approx(620.0));
  REQUIRE(d_mgi.heightInPixels(d_mgi.bounds(), d_mgi.srs()) == Approx(350.0));
  REQUIRE(d_mgi.widthInPixels(util::nonExactBoundsTransform(d_mgi.bounds(), d_mgi.srs(), webmercator), webmercator) == Approx(620.0));
  REQUIRE(d_mgi.heightInPixels(util::nonExactBoundsTransform(d_mgi.bounds(), d_mgi.srs(), webmercator), webmercator) == Approx(350.0));


  auto adjust_bounds = [](auto bounds) {
    const auto unadjusted_width = bounds.getWidth();
    const auto unadjusted_height = bounds.getHeight();
    bounds.setMinX(bounds.getMinX() + unadjusted_width * 0.2);
    bounds.setMinY(bounds.getMinY() + unadjusted_height * 0.3);
    bounds.setMaxX(bounds.getMaxX() - unadjusted_width * 0.1);
    bounds.setMaxY(bounds.getMaxY() - unadjusted_height * 0.2);
    return bounds;
  };

  REQUIRE(d_wgs84.widthInPixels(adjust_bounds(d_wgs84.bounds()), d_wgs84.srs()) == Approx(620.0 * 0.7));
  REQUIRE(d_wgs84.heightInPixels(adjust_bounds(d_wgs84.bounds()), d_wgs84.srs()) == Approx(350.0 * 0.5));

  const auto webmercator_bounds = util::nonExactBoundsTransform(d_wgs84.bounds(), d_wgs84.srs(), webmercator);
  REQUIRE(d_wgs84.widthInPixels(webmercator_bounds, webmercator) == Approx(620.0));
  REQUIRE(d_wgs84.heightInPixels(webmercator_bounds, webmercator) == Approx(350.0));

  const auto webmercator_adjusted_bounds = adjust_bounds(webmercator_bounds);
  REQUIRE(d_wgs84.widthInPixels(webmercator_adjusted_bounds, webmercator) == Approx(620.0 * 0.7));
  REQUIRE(d_wgs84.heightInPixels(webmercator_adjusted_bounds, webmercator) == Approx(350.0 * 0.5));
}

TEST_CASE("reading") {
  const std::vector at100m = {
    "/austria/at_100m_mgi.tif",
    "/austria/at_100m_epsg3857.tif",
    "/austria/at_100m_epsg4326.tif",
    "/austria/at_x149m_y100m_epsg4326.tif",};

  const std::vector vienna20m = {
    "/austria/vienna_20m_mgi.tif",
    "/austria/vienna_20m_epsg3857.tif",
    "/austria/vienna_20m_epsg4326.tif",};

  const std::vector tauern10m = {
    "/austria/tauern_10m_mgi.tif",
    "/austria/tauern_10m_epsg3857.tif",
    "/austria/tauern_10m_epsg4326.tif",};

  const std::vector pizbuin1m = {
    "/austria/pizbuin_1m_mgi.tif",
    "/austria/pizbuin_1m_epsg3857.tif",
    "/austria/pizbuin_1m_epsg4326.tif",};

  OGRSpatialReference geodetic_srs;
  geodetic_srs.importFromEPSG(4326);
  geodetic_srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  SECTION("check for min and max heights") {
    const std::array test_srses = {4326, 3857};
    const std::array test_locations = {
      // CRS bounds, [lower limit, at least one value smaller, at least one value larger, upper limit]
#ifdef ATB_UNITTESTS_EXTENDED
      std::make_tuple(
        "at100m",
        at100m,
        ctb::CRSBounds(9.5, 46.4, 17.1, 49.0),
        std::make_tuple(100.0f, 120.0f, 3000.0f, 3800.0f)), // Austria is between 115 and 3798m
      std::make_tuple(
        "vienna20m",
        vienna20m,
        ctb::CRSBounds(16.17, 48.14, 16.59, 48.33),
        std::make_tuple(140.0f, 180.0f, 500.0f, 560.0f)),   // vienna, between 151 and 542m
#endif
      std::make_tuple(
        "tauern10m",
        tauern10m,
        ctb::CRSBounds(12.6934117,47.0739300, 12.6944580,47.0748649),
        std::make_tuple(3700.0f, 3720.0f, 3790.0f, 3800.0f)), // Grossglockner, 3798m with some surroundings
    };

    for (const auto& test : test_locations) {
      const auto [test_name, test_datasets, geodetic_bounds, limits] = test;

      for (std::string dataset_name : test_datasets) {
        const auto dataset = std::make_shared<Dataset>(ATB_TEST_DATA_DIR + std::string(dataset_name));
        for (const auto& test_srs : test_srses) {
          OGRSpatialReference srs;
          srs.importFromEPSG(test_srs);
          srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

          const auto srs_bounds = util::nonExactBoundsTransform(geodetic_bounds, geodetic_srs, srs);

          const auto reader = DatasetReader(dataset, srs, 1);
          if (ATB_UNITTESTS_DEBUG_IMAGES) {
            const auto heights = reader.read(srs_bounds, 1000, 1000);
            const auto s = std::string("/austria/").length();
            const auto l = dataset_name.length() - std::string(".tif").length() - s;
            const auto debug_file = fmt::format("./heights_{}_srs{}_{}.png", test_name, test_srs, dataset_name.substr(s, l));
            image::debugOut(heights, debug_file);
          }

          const auto heights = reader.read(srs_bounds, 100, 111);
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
    const std::array test_data= {
#ifdef ATB_UNITTESTS_EXTENDED
      std::make_tuple(
        "at100m",
        at100m,
        ctb::CRSBounds(9.5, 46.4, 17.1, 49.0),
        620U, 350U, 42.0, 22.0),
#endif
      std::make_tuple(
        "pizbuin1m",
        pizbuin1m,
        ctb::CRSBounds(10.105646780, 46.839864531, 10.129815588, 46.847626067),
        740U, 315U, 3.01, 0.006),
#ifdef ATB_UNITTESTS_EXTENDED
      std::make_tuple(
        "pizbuin1m_highres",
        pizbuin1m,
        ctb::CRSBounds(10.105646780, 46.839864531, 10.129815588, 46.847626067),
        2000U, 850U, 10.0, 0.008),
#endif
      };


    for (const auto& test : test_data) {
      auto [test_name, datasets, ref_bounds, render_width, render_height, max_abs_diff, max_mse] = test;

      const auto ref_dataset = std::make_shared<Dataset>(ATB_TEST_DATA_DIR + std::string(datasets.front()));
      const auto ref_reader = DatasetReader(ref_dataset, geodetic_srs, 1);
      const auto ref_heights = ref_reader.read(ref_bounds, render_width, render_height);
      if (ATB_UNITTESTS_DEBUG_IMAGES)
        image::debugOut(ref_heights, fmt::format("./heights_ref.png"));

      for (std::string dataset_name : datasets) {
        const auto dataset = std::make_shared<Dataset>(ATB_TEST_DATA_DIR + std::string(dataset_name));
        const auto reader = DatasetReader(dataset, geodetic_srs, 1);
        const auto heights = reader.read(ref_bounds, render_width, render_height);

        const auto s = std::string("/austria/").length();
        const auto l = dataset_name.length() - std::string(".tif").length() - s;

        if (ATB_UNITTESTS_DEBUG_IMAGES) {
          image::debugOut(ref_heights, fmt::format("./heights_{}_{}.png", test_name, dataset_name.substr(s, l)));

          auto height_diffs = HeightData(render_width, render_height);
          std::transform(ref_heights.begin(), ref_heights.end(), heights.begin(), height_diffs.begin(), [](auto a, auto b) { return std::abs(a-b); });
          const auto path = fmt::format("./diffs_{}_{}.png", test_name, dataset_name.substr(s, l));
          image::debugOut(height_diffs, path);
        }
        auto largest_abs_diff = 0.0;
        const auto mse = std::transform_reduce(ref_heights.begin(), ref_heights.end(), heights.begin(), 0.0, std::plus<>(), [&largest_abs_diff](auto a, auto b) {
                           const auto t = std::abs(double(a)-double(b));
                           largest_abs_diff = std::max(t, largest_abs_diff);
                           return t * t;
                         }) / double(ref_heights.size());
//        fmt::print("{} | {};  mse: {}, largest_abs_diff: {}\n", test_name, dataset_name.substr(s, l), mse, largest_abs_diff);
        CHECK(largest_abs_diff < double(max_abs_diff));
        CHECK(mse < max_mse);
      }
    }

  }

  SECTION("overview without warping") {
    const auto border = 5;
    const auto low_res_ds = std::make_shared<Dataset>(ATB_TEST_DATA_DIR "/austria/at_100m_mgi.tif");
    const auto high_res_ds = std::make_shared<Dataset>("/home/madam/valtava/raw/Oe_2020/OeRect_01m_gt_31287.img");
    const auto srs = low_res_ds->srs();

    const auto low_res_reader = DatasetReader(low_res_ds, srs, 1);
    const auto high_res_reader = DatasetReader(high_res_ds, srs, 1);
    const auto render_width = low_res_ds->widthInPixels() - 2 * border;
    const auto render_height = low_res_ds->heightInPixels() - 2 * border;

    // the error is not 0, apparently there is still some resampling going on. the diff image at least looks like sampling artifacts.
    // in theory, gdalwarp should do the exact same thing as this unit test, but for some reason this is not the case. That's also
    // indicated by the errors on the borders (which we are cutting away).
    // i'm ignoring it for now, as the error seems low enouugh.
    const auto max_abs_diff = 18.0;
    const auto max_mse = 0.6;

    const auto pixel_width = low_res_ds->pixelWidthIn(srs);
    const auto pixel_height = low_res_ds->pixelHeightIn(srs);
    auto srs_bounds = util::nonExactBoundsTransform(low_res_ds->bounds(), low_res_ds->srs(), srs);
    srs_bounds.setBounds(srs_bounds.getMinX() + border * pixel_width, srs_bounds.getMinY() + border * pixel_height, srs_bounds.getMaxX() - border * pixel_width, srs_bounds.getMaxY() - border * pixel_height);

    const auto t0 = std::chrono::high_resolution_clock::now();
    const auto low_res_heights = low_res_reader.read(srs_bounds, render_width, render_height);
    const auto t1 = std::chrono::high_resolution_clock::now();
    const auto high_res_heights = high_res_reader.readWithOverviews(srs_bounds, render_width, render_height);
    const auto t2 = std::chrono::high_resolution_clock::now();

    REQUIRE(low_res_heights.width() == render_width);
    REQUIRE(low_res_heights.height() == render_height);
    REQUIRE(high_res_heights.width() == render_width);
    REQUIRE(high_res_heights.height() == render_height);

    const auto low_res_time = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    const auto high_res_time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    fmt::print("low res time: {}; high res time: {}\n", double(low_res_time) / 1000.0, double(high_res_time) / 1000.0);

    if (ATB_UNITTESTS_DEBUG_IMAGES) {
      image::debugOut(low_res_heights, fmt::format("./low_res_heights.png"));
      image::debugOut(high_res_heights, fmt::format("./high_res_heights.png"));

      auto height_diffs = HeightData(render_width, render_height);
      std::transform(low_res_heights.begin(), low_res_heights.end(), high_res_heights.begin(), height_diffs.begin(), [](auto a, auto b) { return std::abs(a-b); });
      image::debugOut(height_diffs, "./diff_low_res_high_res.png");
    }
    auto largest_abs_diff = 0.0;
    const auto mse = std::transform_reduce(low_res_heights.begin(), low_res_heights.end(), high_res_heights.begin(), 0.0, std::plus<>(), [&largest_abs_diff](auto a, auto b) {
                       const auto t = std::abs(double(a)-double(b));
                       largest_abs_diff = std::max(t, largest_abs_diff);
                       return t * t;
                     }) / double(low_res_heights.size());
//    fmt::print("mse: {}, largest_abs_diff: {}\n", mse, largest_abs_diff);
    CHECK(largest_abs_diff < double(max_abs_diff));
    CHECK(mse < max_mse);
  }


  SECTION("overview with warping") {
    const auto low_res_ds = std::make_shared<Dataset>(ATB_TEST_DATA_DIR "/austria/at_100m_epsg4326.tif");
    const auto high_res_ds = std::make_shared<Dataset>("/home/madam/valtava/raw/Oe_2020/OeRect_01m_gt_31287.img");
    const auto srs = low_res_ds->srs();
    const auto low_res_reader = DatasetReader(low_res_ds, srs, 1);
    const auto high_res_reader = DatasetReader(high_res_ds, srs, 1);
    const auto srs_bounds = util::nonExactBoundsTransform(ctb::CRSBounds(9.5, 46.4, 17.1, 49.0), geodetic_srs, srs);


    const auto render_width = unsigned(low_res_ds->widthInPixels(srs_bounds, srs));
    const auto render_height = unsigned(low_res_ds->heightInPixels(srs_bounds, srs));
    const auto max_abs_diff = 34.0;
    const auto max_mse = 1.2;


    const auto t0 = std::chrono::high_resolution_clock::now();
    const auto low_res_heights = low_res_reader.read(srs_bounds, render_width, render_height);
    const auto t1 = std::chrono::high_resolution_clock::now();
    const auto high_res_heights = high_res_reader.readWithOverviews(srs_bounds, render_width, render_height);
    const auto t2 = std::chrono::high_resolution_clock::now();

    REQUIRE(low_res_heights.width() == render_width);
    REQUIRE(low_res_heights.height() == render_height);
    REQUIRE(high_res_heights.width() == render_width);
    REQUIRE(high_res_heights.height() == render_height);

    const auto low_res_time = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    const auto high_res_time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
//    fmt::print("low res time: {}; high res time: {}\n", double(low_res_time) / 1000.0, double(high_res_time) / 1000.0);

    if (ATB_UNITTESTS_DEBUG_IMAGES) {
      image::debugOut(low_res_heights, fmt::format("./ov_with_warping_low_res_heights.png"));
      image::debugOut(high_res_heights, fmt::format("./ov_with_warping_high_res_heights.png"));

      auto height_diffs = HeightData(render_width, render_height);
      std::transform(low_res_heights.begin(), low_res_heights.end(), high_res_heights.begin(), height_diffs.begin(), [](auto a, auto b) { return std::abs(a-b); });
      image::debugOut(height_diffs, "./ov_with_warping_diff_low_res_high_res.png");
    }
    auto largest_abs_diff = 0.0;
    const auto mse = std::transform_reduce(low_res_heights.begin(), low_res_heights.end(), high_res_heights.begin(), 0.0, std::plus<>(), [max_abs_diff=max_abs_diff, &largest_abs_diff](auto a, auto b) {
                       const auto t = std::abs(double(a)-double(b));
                       largest_abs_diff = std::max(t, largest_abs_diff);
                       return t * t;
                     }) / double(low_res_heights.size());
//    fmt::print("mse: {}, largest_abs_diff: {}\n", mse, largest_abs_diff);
    CHECK(largest_abs_diff < double(max_abs_diff));
    CHECK(mse < max_mse);
  }
}


