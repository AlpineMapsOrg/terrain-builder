#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>
#include <radix/tile.h>

#include "../opencv_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "io/bytes.h"
#include "texture/io.h"
#include "TileProvider.h"

TEST_CASE("A static provider gets the image of a tile it has", "[tile_provider]") {
    const radix::tile::Id tile(1, {0, 1});
    const cv::Mat image(1, 1, CV_8UC3, cv::Vec3b(0, 0, 255));
    const StaticTileProvider provider({{tile, image}});

    CHECK(provider.has_tile(tile));
    const cv::Mat got = REQUIRE_VALUE(provider.get_tile(tile));
    CHECK(mat_equals(got, image));
}

TEST_CASE("A static provider fails to get a tile it does not have, naming the tile", "[tile_provider]") {
    const radix::tile::Id tile(1, {0, 1});
    const radix::tile::Id other(1, {1, 1});
    const StaticTileProvider provider({{tile, cv::Mat(1, 1, CV_8UC3, cv::Vec3b(0, 0, 255))}});

    CHECK(!provider.has_tile(other));
    const TileError error = REQUIRE_ERROR(provider.get_tile(other));
    CHECK(error.holds(MissingTile{other}));
}

TEST_CASE("An empty provider fails to get any tile, naming the tile", "[tile_provider]") {
    const radix::tile::Id tile(1, {0, 1});
    const EmptyTileProvider provider;

    CHECK(!provider.has_tile(tile));
    const TileError error = REQUIRE_ERROR(provider.get_tile(tile));
    CHECK(error.holds(MissingTile{tile}));
}

TEST_CASE("A zoom range provider has only the tiles of its inner provider that are in range", "[tile_provider]") {
    const radix::tile::Id below(0, {0, 0});
    const radix::tile::Id inside(1, {0, 1});
    const cv::Mat image(1, 1, CV_8UC3, cv::Vec3b(0, 0, 255));
    const ZoomRangeTileProvider<StaticTileProvider> provider(StaticTileProvider({{below, image}, {inside, image}}), 1, std::nullopt);

    CHECK(provider.has_tile(inside));
    const cv::Mat got = REQUIRE_VALUE(provider.get_tile(inside));
    CHECK(mat_equals(got, image));

    CHECK(!provider.has_tile(below));
    const TileError error = REQUIRE_ERROR(provider.get_tile(below));
    CHECK(error.holds(MissingTile{below}));
}

TEST_CASE("A path provider gets the image in a tile's file", "[tile_provider]") {
    const TempDirectory directory("tile_provider_file");
    const GoogleMapboxTilePathProvider provider(directory.path());
    const radix::tile::Id tile(1, {0, 1});
    const std::filesystem::path path = provider.get_tile_path(tile).value();
    REQUIRE_VALUE(texture::io::write_to_path(cv::Mat(3, 4, CV_8UC3, cv::Scalar(10, 20, 30)), path));

    CHECK(provider.has_tile(tile));
    const cv::Mat got = REQUIRE_VALUE(provider.get_tile(tile));
    CHECK(got.size() == cv::Size(4, 3));
}

TEST_CASE("A path provider fails to get a tile whose file is missing, naming the tile", "[tile_provider]") {
    const TempDirectory directory("tile_provider_missing");
    const GoogleMapboxTilePathProvider provider(directory.path());
    const radix::tile::Id tile(1, {0, 1});

    CHECK(!provider.has_tile(tile));
    const TileError error = REQUIRE_ERROR(provider.get_tile(tile));
    CHECK(error.holds(MissingTile{tile}));
}

TEST_CASE("A path provider fails to get a tile whose file is not an image with a decode error", "[tile_provider]") {
    const TempDirectory directory("tile_provider_not_an_image");
    const GoogleMapboxTilePathProvider provider(directory.path());
    const radix::tile::Id tile(1, {0, 1});
    const std::filesystem::path path = provider.get_tile_path(tile).value();
    REQUIRE_VALUE(io::write_bytes_to_path(std::vector<uint8_t>{1, 2, 3, 4}, path));

    CHECK(provider.has_tile(tile));
    const TileError error = REQUIRE_ERROR(provider.get_tile(tile));
    CHECK(error.holds<texture::io::DecodeError>());
}
