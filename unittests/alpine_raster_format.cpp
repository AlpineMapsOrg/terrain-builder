#include "AlpineRasterGenerator.h"
#include <algorithm>
#include <filesystem>
#include <functional>
#include <map>
#include <numeric>
#include <ranges>

#include <catch2/catch.hpp>
#include <fmt/core.h>
#include <glm/glm.hpp>
#include <type_traits>

#include "Dataset.h"
#include "Image.h"
#include "Tiler.h"
#include "catch2_helpers.h"
#include "ctb/Grid.hpp"
#include "ctb/types.hpp"

TEST_CASE("conversion math") {
  const auto one_red = 32.0f;
  const auto one_green = 32.000000001f / 256;
  const auto eps = 0.000000001f;

  CHECK(AlpineRasterGenerator::convert(0) == glm::u8vec3(0, 0, 0));

  CHECK(AlpineRasterGenerator::convert(one_red + eps) == glm::u8vec3(1, 0, 0));
  CHECK(AlpineRasterGenerator::convert(one_green + eps) == glm::u8vec3(0, 1, 0));

  CHECK(AlpineRasterGenerator::convert(42 * one_red + eps) == glm::u8vec3(42, 0, 0));
  CHECK(AlpineRasterGenerator::convert(200 * one_green + eps) == glm::u8vec3(0, 200, 0));


  CHECK(AlpineRasterGenerator::convert(255 * one_red + eps) == glm::u8vec3(255, 0, 0));
  CHECK(AlpineRasterGenerator::convert(253 * one_green + eps) == glm::u8vec3(0, 253, 0));
  CHECK(AlpineRasterGenerator::convert(254 * one_green + eps) == glm::u8vec3(0, 254, 0));
  CHECK(AlpineRasterGenerator::convert(255 * one_green + eps) == glm::u8vec3(0, 255, 0));
  CHECK(AlpineRasterGenerator::convert(256 * one_green + eps) == glm::u8vec3(1, 0, 0));
  CHECK(AlpineRasterGenerator::convert(257 * one_green + eps) == glm::u8vec3(1, 1, 0));
  CHECK(AlpineRasterGenerator::convert(258 * one_green + eps) == glm::u8vec3(1, 2, 0));

  CHECK(AlpineRasterGenerator::convert(240 * one_red + 195 * one_green + eps) == glm::u8vec3(240, 195, 0));
  CHECK(AlpineRasterGenerator::convert(64 * one_red + 255 * one_green + eps) == glm::u8vec3(64, 255, 0));
  CHECK(AlpineRasterGenerator::convert(255 * one_red + 128 * one_green + eps) == glm::u8vec3(255, 128, 0));
  CHECK(AlpineRasterGenerator::convert(255 * one_red + 255 * one_green + eps) == glm::u8vec3(255, 255, 0));

  CHECK(AlpineRasterGenerator::convert(123 * one_red + 250 * one_green + eps) == glm::u8vec3(123, 250, 0));
  CHECK(AlpineRasterGenerator::convert(140 * one_red + 255 * one_green + eps) == glm::u8vec3(140, 255, 0));
  CHECK(AlpineRasterGenerator::convert(141 * one_red + 0 * one_green + eps) == glm::u8vec3(141, 0, 0));
  CHECK(AlpineRasterGenerator::convert(141 * one_red + 1 * one_green + eps) == glm::u8vec3(141, 1, 0));
}

Tiler::Border testTypeValue2Border(bool v) {
  return v ? Tiler::Border::Yes : Tiler::Border::No;
}

TEMPLATE_TEST_CASE("alpine raster format, border ", "", std::true_type, std::false_type) {
  std::filesystem::remove_all("./unittest_tiles/");

  SECTION("raster conversion: 0 heights") {
    RgbImage t = AlpineRasterGenerator::convertHeights(HeightData(256, 256));
    REQUIRE(t.width() == 256);
    REQUIRE(t.height() == 256);
    CHECK(std::ranges::all_of(t, [](const auto& v) { return v == glm::u8vec3(0, 0, 0); }));
  }

  SECTION("raster conversion: random heights") {
    std::mt19937 random_engine(0);
    std::uniform_real_distribution<float> dist(0.f, 8000.f);
    HeightData heights(256, 256);
    for (auto& h : heights) {
      h = dist(random_engine);
    }

    RgbImage t = AlpineRasterGenerator::convertHeights(heights);
    REQUIRE(t.width() == 256);
    REQUIRE(t.height() == 256);
    const auto check = [](const auto& t, const auto& h) {
      return t == AlpineRasterGenerator::convert(h);
    };
    CHECK(std::transform_reduce(t.begin(), t.end(), heights.begin(), true, std::logical_and<>(), check) == true);
  }

  SECTION("raste write") {
    const auto generator = AlpineRasterGenerator::make("./unittest_tiles/", ATB_TEST_DATA_DIR "/austria/at_mgi.tif", ctb::Grid::Srs::SphericalMercator, Tiler::Scheme::Tms, Tiler::Border::Yes);
    generator.write(ctb::TilePoint(0, 0), 0, HeightData(257, 257));
    CHECK(std::filesystem::exists("./unittest_tiles/0/0/0.png"));

    generator.write(ctb::TilePoint(2, 3), 1, HeightData(257, 257));
    CHECK(std::filesystem::exists("./unittest_tiles/1/2/3.png"));

    // check that a second write doesn't crash
    generator.write(ctb::TilePoint(2, 3), 1, HeightData(257, 257));
    CHECK(std::filesystem::exists("./unittest_tiles/1/2/3.png"));

    // in the best case, we would read back the data and check it. but that's too much work for now.
    // it'll be checked in cesium.js
  }

  SECTION("list tiles") {
    const auto generator = AlpineRasterGenerator::make("./unittest_tiles/", ATB_TEST_DATA_DIR "/austria/at_mgi.tif", ctb::Grid::Srs::SphericalMercator, Tiler::Scheme::Tms, testTypeValue2Border(TestType::value));
    const auto tiles = generator.listTiles();
    CHECK(!tiles.empty());
    std::map<ctb::i_zoom, unsigned> n_tiles;
    for (const auto& t : tiles) {
      n_tiles[t.zoom]++;
    }
    REQUIRE(n_tiles.size() == 7);
    CHECK(n_tiles[0] == 1);
    CHECK(n_tiles[1] == 1);
    CHECK(n_tiles[2] == 1);
    CHECK(n_tiles[3] == 1);
    CHECK(n_tiles[4] == 1);
    CHECK(n_tiles[5] == 4);
    CHECK(n_tiles[6] == 6);

    const auto check = [](const Tile& t) {
      return t.tileSize == (256 + unsigned(testTypeValue2Border(TestType::value))) && t.gridSize == 256;
    };
    CHECK(std::transform_reduce(tiles.begin(), tiles.end(), true, std::logical_and<>(), check) == true);
  }

  SECTION("list tiles with given zoom level") {
    const auto generator = AlpineRasterGenerator::make("./unittest_tiles/", ATB_TEST_DATA_DIR "/austria/at_mgi.tif", ctb::Grid::Srs::SphericalMercator, Tiler::Scheme::Tms, testTypeValue2Border(TestType::value));
    for (ctb::i_zoom max = 1; max < 10; ++max) {
      const auto tiles = generator.listTiles(max);
      CHECK(!tiles.empty());
      std::map<ctb::i_zoom, unsigned> n_tiles;
      for (const auto& t : tiles) {
        n_tiles[t.zoom]++;
      }
      CHECK(n_tiles.size() == max);
    }
  }

  SECTION("process all tiles") {
    const auto generator = AlpineRasterGenerator::make("./unittest_tiles/", ATB_TEST_DATA_DIR "/austria/at_mgi.tif", ctb::Grid::Srs::SphericalMercator, Tiler::Scheme::Tms, testTypeValue2Border(TestType::value));
    generator.process();
    const auto tiles = generator.listTiles();

    const auto check = [](const Tile& t) {
      return std::filesystem::exists(fmt::format("./unittest_tiles/{}/{}/{}.png", t.zoom, t.point.x, t.point.y));
    };
    CHECK(std::transform_reduce(tiles.begin(), tiles.end(), true, std::logical_and<>(), check) == true);
  }
#if defined(ATB_UNITTESTS_EXTENDED) && ATB_UNITTESTS_EXTENDED
  SECTION("process all tiles with max zoom") {
    const auto generator = AlpineRasterGenerator::make("./unittest_tiles/", ATB_TEST_DATA_DIR "/austria/at_mgi.tif", ctb::Grid::Srs::SphericalMercator, Tiler::Scheme::Tms, testTypeValue2Border(TestType::value));
    generator.process(8);
    const auto tiles = generator.listTiles(8);

    const auto check = [](const Tile& t) {
      return std::filesystem::exists(fmt::format("./unittest_tiles/{}/{}/{}.png", t.zoom, t.point.x, t.point.y));
    };
    CHECK(std::transform_reduce(tiles.begin(), tiles.end(), true, std::logical_and<>(), check) == true);
  }
#endif
}
