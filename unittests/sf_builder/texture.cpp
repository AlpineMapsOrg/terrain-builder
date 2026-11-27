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
#include <cmath>
#include <filesystem>
#include <limits>
#include <optional>
#include <ranges>
#include <set>

#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "error/utils.h"
#include "gis/webmercator.h"
#include "io/bytes.h"

#include "mesh/SimpleMesh.h"
#include "texture_assembler.h"

TEST_CASE("estimate_zoom_level", "[texture_assembler]") {
    const radix::tile::Id tile(20, glm::uvec2(0, 1));
    const radix::tile::SrsBounds tile_bounds = gis::webmercator::compute_tile_bounds(tile);
    const radix::tile::SrsBounds shifted_bounds(tile_bounds.min + glm::dvec2(-100, 420), tile_bounds.max + glm::dvec2(-100, 420));

    CHECK(sf::estimate_zoom_level(tile.zoom_level, tile_bounds, shifted_bounds) == tile.zoom_level);
}

class AvailabilityListEmptyTileProvider : public TileProvider {
public:
    AvailabilityListEmptyTileProvider(std::set<radix::tile::Id> available_tiles)
        : available_tiles(available_tiles) {
    }

    virtual bool has_tile(const radix::tile::Id tile) const override {
        return this->available_tiles.find(tile) != this->available_tiles.end();
    }

    virtual alp::Result<cv::Mat, TileError> get_tile(const radix::tile::Id tile) const override {
        if (!this->has_tile(tile)) {
            return alp::make_error(MissingTile{tile});
        }
        return cv::Mat();
    }

private:
    const std::set<radix::tile::Id> available_tiles;
};

class AlwaysEmptyTileProvider : public TileProvider {
public:
    virtual bool has_tile(const radix::tile::Id) const override {
        return true;
    }

    virtual alp::Result<cv::Mat, TileError> get_tile(const radix::tile::Id) const override {
        return cv::Mat();
    }
};

TEST_CASE("texture assembler takes root tile if only available ", "[texture_assembler]") {
    const radix::tile::Id root_tile(3, glm::uvec2(5, 4));
    const std::set<radix::tile::Id> available_tiles = {
        {3, {5, 4}}};
    const std::set<radix::tile::Id> expected_tiles = {
        {3, {5, 4}}};

    const AvailabilityListEmptyTileProvider tile_provider(available_tiles);

    std::vector<radix::tile::Id> actual_tiles_vec = sf::find_relevant_tiles_to_splatter_in_bounds(
        root_tile,
        gis::webmercator::compute_tile_bounds(root_tile),
        tile_provider);
    std::set<radix::tile::Id> actual_tiles(std::make_move_iterator(actual_tiles_vec.begin()),
                                    std::make_move_iterator(actual_tiles_vec.end()));

    CHECK(expected_tiles == actual_tiles);
}

TEST_CASE("texture assembler ignores parent if all children are present", "[texture_assembler]") {
    const radix::tile::Id root_tile(3, glm::uvec2(5, 4));
    const std::set<radix::tile::Id> available_tiles = {
        {3, {5, 4}},
        {4, {10, 8}},
        {4, {11, 8}},
        {4, {10, 9}},
        {4, {11, 9}}};
    const std::set<radix::tile::Id> expected_tiles = {
        {4, {10, 8}},
        {4, {11, 8}},
        {4, {10, 9}},
        {4, {11, 9}}};

    const AvailabilityListEmptyTileProvider tile_provider(available_tiles);

    std::vector<radix::tile::Id> actual_tiles_vec = sf::find_relevant_tiles_to_splatter_in_bounds(
        root_tile,
        gis::webmercator::compute_tile_bounds(root_tile),
        tile_provider);
    std::set<radix::tile::Id> actual_tiles(std::make_move_iterator(actual_tiles_vec.begin()),
                                    std::make_move_iterator(actual_tiles_vec.end()));

    CHECK(expected_tiles == actual_tiles);
}

TEST_CASE("texture assembler considers max zoom level", "[texture_assembler]") {
    const radix::tile::Id root_tile(3, glm::uvec2(5, 4));
    const std::set<radix::tile::Id> available_tiles = {
        {3, {5, 4}},
        {4, {10, 8}},
        {4, {11, 8}},
        {4, {10, 9}},
        {4, {11, 9}}};
    const std::set<radix::tile::Id> expected_tiles = {
        {3, {5, 4}}};

    const AvailabilityListEmptyTileProvider tile_provider(available_tiles);

    std::vector<radix::tile::Id> actual_tiles_vec = sf::find_relevant_tiles_to_splatter_in_bounds(
        root_tile,
        gis::webmercator::compute_tile_bounds(root_tile),
        tile_provider,
        3);
    std::set<radix::tile::Id> actual_tiles(std::make_move_iterator(actual_tiles_vec.begin()),
                                    std::make_move_iterator(actual_tiles_vec.end()));

    CHECK(expected_tiles == actual_tiles);
}

TEST_CASE("texture assembler works for arbitrary bounds", "[texture_assembler]") {
    const std::set<radix::tile::Id> available_tiles = {
        // {21, {1048576, 1048575}},
        {21, {1048577, 1048575}},
        {21, {1048578, 1048575}},
        {21, {1048579, 1048575}},
        {21, {1048580, 1048575}},
        // {21, {1048581, 1048575}},
        {20, {524288, 524287}},
        {20, {524289, 524287}},
        {20, {524290, 524287}},
        // {19, {262144, 262143}},
        {19, {262145, 262143}}};
    const std::set<radix::tile::Id> expected_tiles = {
        // {21, {1048576, 1048575}},
        {21, {1048577, 1048575}},
        {21, {1048578, 1048575}},
        {21, {1048579, 1048575}},
        {21, {1048580, 1048575}},
        // {21, {1048581, 1048575}},
        {20, {524288, 524287}},
        // {20, {524289, 524287}},
        {20, {524290, 524287}},
        // {19, {262144, 262143}},
        // {19, {262145, 262143}}
    };

    const AvailabilityListEmptyTileProvider tile_provider(available_tiles);

    const radix::tile::SrsBounds target_bounds(glm::dvec2(0, 0), glm::dvec2(100, 1));
    const radix::tile::Id root_tile = sf::find_smallest_encompassing_tile(target_bounds).value();
    std::vector<radix::tile::Id> actual_tiles_vec = sf::find_relevant_tiles_to_splatter_in_bounds(
        root_tile,
        target_bounds,
        tile_provider,
        23,
        21);
    std::set<radix::tile::Id> actual_tiles(std::make_move_iterator(actual_tiles_vec.begin()),
                                    std::make_move_iterator(actual_tiles_vec.end()));

    CHECK(expected_tiles == actual_tiles);
}

TEST_CASE("texture assembler does not fail if there are not tiles", "[texture_assembler]") {
    const std::set<radix::tile::Id> available_tiles = {};
    const std::set<radix::tile::Id> expected_tiles = {};

    const AvailabilityListEmptyTileProvider tile_provider(available_tiles);

    const radix::tile::SrsBounds target_bounds(glm::dvec2(0, 0), glm::dvec2(100, 1));
    const radix::tile::Id root_tile = sf::find_smallest_encompassing_tile(target_bounds).value();
    std::vector<radix::tile::Id> actual_tiles_vec = sf::find_relevant_tiles_to_splatter_in_bounds(
        root_tile,
        target_bounds,
        tile_provider);
    std::set<radix::tile::Id> actual_tiles(std::make_move_iterator(actual_tiles_vec.begin()),
                                    std::make_move_iterator(actual_tiles_vec.end()));

    CHECK(expected_tiles == actual_tiles);
}

TEST_CASE("texture assembler assembles single tile", "[texture_assembler]") {
    const std::unordered_map<radix::tile::Id, cv::Mat, radix::tile::Id::Hasher> tiles_to_texture = {
        {radix::tile::Id(0, {0, 0}), cv::Mat(1, 1, CV_8UC3, cv::Vec3b(0, 0, 255))},
    };
    std::vector<radix::tile::Id> tiles_to_splatter;
    std::transform(tiles_to_texture.begin(), tiles_to_texture.end(), std::back_inserter(tiles_to_splatter),
                   [](const auto &pair) { return pair.first; });

    const StaticTileProvider tile_provider(tiles_to_texture);

    const radix::tile::Id root_tile(0, {0, 0});
    cv::Mat assembled_texture = REQUIRE_VALUE(sf::splatter_tiles_to_texture(
        root_tile,
        gis::webmercator::compute_tile_bounds(root_tile),
        tile_provider,
        tiles_to_splatter,
        cv::INTER_NEAREST_EXACT)).image;

    REQUIRE(assembled_texture.size() == cv::Size(1, 1));
    CHECK(assembled_texture.type() == CV_8UC3);

    CHECK(assembled_texture.at<cv::Vec3b>(0, 0) == cv::Vec3b(0, 0, 255));
}

TEST_CASE("texture assembler assembles two tiles", "[texture_assembler]") {
    const std::unordered_map<radix::tile::Id, cv::Mat, radix::tile::Id::Hasher> tiles_to_texture = {
        {radix::tile::Id(1, {0, 0}), cv::Mat(1, 1, CV_8UC3, cv::Vec3b(0, 0, 255))},
        {radix::tile::Id(1, {0, 1}), cv::Mat(1, 1, CV_8UC3, cv::Vec3b(0, 255, 0))},
    };
    std::vector<radix::tile::Id> tiles_to_splatter;
    std::transform(tiles_to_texture.begin(), tiles_to_texture.end(), std::back_inserter(tiles_to_splatter),
                   [](const auto &pair) { return pair.first; });

    const StaticTileProvider tile_provider(tiles_to_texture);

    const radix::tile::Id root_tile(0, {0, 0});
    cv::Mat assembled_texture = REQUIRE_VALUE(sf::splatter_tiles_to_texture(
        root_tile,
        gis::webmercator::compute_tile_bounds(root_tile),
        tile_provider,
        tiles_to_splatter,
        cv::INTER_NEAREST_EXACT)).image;

    REQUIRE(assembled_texture.size() == cv::Size(2, 2));
    CHECK(assembled_texture.type() == CV_8UC3);

    CHECK(assembled_texture.at<cv::Vec3b>(0, 0) == cv::Vec3b(0, 255, 0));
    CHECK(assembled_texture.at<cv::Vec3b>(1, 0) == cv::Vec3b(0, 0, 255));
}

TEST_CASE("texture assembler correct order of texture writes", "[texture_assembler]") {
    const std::unordered_map<radix::tile::Id, cv::Mat, radix::tile::Id::Hasher> tiles_to_texture = {
        {radix::tile::Id(0, {0, 0}), cv::Mat(1, 1, CV_8UC1, uint8_t(1))},
        {radix::tile::Id(1, {0, 0}), cv::Mat(1, 1, CV_8UC1, uint8_t(2))},
        {radix::tile::Id(1, {0, 1}), cv::Mat(1, 1, CV_8UC1, uint8_t(3))},
    };
    std::vector<radix::tile::Id> tiles_to_splatter;
    std::transform(tiles_to_texture.begin(), tiles_to_texture.end(), std::back_inserter(tiles_to_splatter),
                   [](const auto &pair) { return pair.first; });
    std::sort(tiles_to_splatter.begin(), tiles_to_splatter.end(),
        [](const radix::tile::Id &a, const radix::tile::Id &b) { return a.zoom_level < b.zoom_level; });

    const StaticTileProvider tile_provider(tiles_to_texture);

    const radix::tile::Id root_tile(0, {0, 0});
    cv::Mat assembled_texture = REQUIRE_VALUE(sf::splatter_tiles_to_texture(
        root_tile,
        gis::webmercator::compute_tile_bounds(root_tile),
        tile_provider,
        tiles_to_splatter,
        cv::INTER_NEAREST_EXACT)).image;

    REQUIRE(assembled_texture.size() == cv::Size(2, 2));
    CHECK(assembled_texture.type() == CV_8UC1);

    CHECK(assembled_texture.at<uint8_t>(0, 0) == uint8_t(3));
    CHECK(assembled_texture.at<uint8_t>(1, 0) == uint8_t(2));
    CHECK(assembled_texture.at<uint8_t>(0, 1) == uint8_t(1));
    CHECK(assembled_texture.at<uint8_t>(1, 1) == uint8_t(1));
}

/// Bounds covering the given pixel range of a 256 pixel root tile, in bottom left origin.
static radix::tile::SrsBounds bounds_of_root_tile_pixels(
    const radix::tile::SrsBounds &root_tile_bounds,
    const glm::uvec2 pixel_min,
    const glm::uvec2 pixel_max) {
    return radix::tile::SrsBounds(
        root_tile_bounds.min + root_tile_bounds.size() * (glm::dvec2(pixel_min) / 256.0),
        root_tile_bounds.min + root_tile_bounds.size() * (glm::dvec2(pixel_max) / 256.0));
}

TEST_CASE("target image region without padding or alignment", "[texture_assembler]") {
    const radix::tile::SrsBounds root_tile_bounds(glm::dvec2(0, 0), glm::dvec2(256, 256));
    const radix::tile::SrsBounds target_bounds = bounds_of_root_tile_pixels(root_tile_bounds, glm::uvec2(100, 70), glm::uvec2(150, 160));

    const sf::TargetImageRegion region = sf::calculate_target_image_region(
        target_bounds, root_tile_bounds, glm::uvec2(256), 0, glm::uvec2(0), 1);

    // Requested region, mapped to top left origin.
    CHECK(region.full.min == glm::u64vec2(100, 96));
    CHECK(region.full.max == glm::u64vec2(150, 186));
    CHECK(region.content.min == glm::uvec2(0, 0));
    CHECK(glm::u64vec2(region.content.max) == region.full.size());
}

TEST_CASE("target image region applies padding and alignment", "[texture_assembler]") {
    const radix::tile::SrsBounds root_tile_bounds(glm::dvec2(0, 0), glm::dvec2(256, 256));
    const radix::tile::SrsBounds target_bounds = bounds_of_root_tile_pixels(root_tile_bounds, glm::uvec2(100, 70), glm::uvec2(150, 160));

    const sf::TargetImageRegion region = sf::calculate_target_image_region(
        target_bounds, root_tile_bounds, glm::uvec2(256), 0, glm::uvec2(2), 16);

    // Requested (100, 96) to (150, 186), padded by 2 and grown to multiples of 16.
    CHECK(region.full.min == glm::u64vec2(96, 80));
    CHECK(region.full.max == glm::u64vec2(160, 192));
    CHECK(region.content.min == glm::uvec2(4, 16));
    CHECK(region.content.max == glm::uvec2(54, 106));
}

TEST_CASE("target image region is clamped to the root tile image", "[texture_assembler]") {
    const radix::tile::SrsBounds root_tile_bounds(glm::dvec2(0, 0), glm::dvec2(256, 256));
    const radix::tile::SrsBounds target_bounds = bounds_of_root_tile_pixels(root_tile_bounds, glm::uvec2(0, 0), glm::uvec2(256, 256));

    const sf::TargetImageRegion region = sf::calculate_target_image_region(
        target_bounds, root_tile_bounds, glm::uvec2(256), 0, glm::uvec2(2), 16);

    CHECK(region.full.min == glm::u64vec2(0, 0));
    CHECK(region.full.max == glm::u64vec2(256, 256));
    CHECK(region.content.min == glm::uvec2(0, 0));
    CHECK(region.content.max == glm::uvec2(256, 256));
}

TEST_CASE("target image region is exact in a root tile image wider than 32 bits", "[texture_assembler]") {
    const radix::tile::SrsBounds root_tile_bounds(glm::dvec2(0, 0), glm::dvec2(256, 256));
    // One pixel of the root tile image 32 levels below the root.
    const double pixel_size = std::ldexp(256.0, -40);
    const radix::tile::SrsBounds target_bounds(glm::dvec2(128), glm::dvec2(128 + 100 * pixel_size));

    const sf::TargetImageRegion region = sf::calculate_target_image_region(
        target_bounds, root_tile_bounds, glm::uvec2(256), 32, glm::uvec2(0), 1);

    const uint64_t half = uint64_t{1} << 39;
    CHECK(region.full.min == glm::u64vec2(half, half - 100));
    CHECK(region.full.max == glm::u64vec2(half + 100, half));
    CHECK(region.content.min == glm::uvec2(0, 0));
    CHECK(region.content.max == glm::uvec2(100, 100));
}

TEST_CASE("pixel tile bounds of a tile 32 levels below the root end at the edge of the root tile image", "[texture_assembler]") {
    const radix::tile::Id root_tile(0, {0, 0});
    const radix::tile::Id tile(32, {std::numeric_limits<uint32_t>::max(), 0});

    const radix::geometry::Aabb2<uint64_t> bounds = sf::calculate_pixel_tile_bounds(tile, root_tile, glm::uvec2(256), 32);

    const uint64_t root_tile_image_width = uint64_t{1} << 40;
    CHECK(bounds.min == glm::u64vec2(root_tile_image_width - 256, 0));
    CHECK(bounds.max == glm::u64vec2(root_tile_image_width, 256));
}

TEST_CASE("drop_coarse_fallback_tiles removes the tiles more than the limit above the deepest one", "[texture_assembler]") {
    std::vector<radix::tile::Id> tiles = {
        radix::tile::Id(0, {0, 0}),
        radix::tile::Id(1, {0, 0}),
        radix::tile::Id(9, {0, 0})};

    sf::drop_coarse_fallback_tiles(tiles, 8);

    CHECK(tiles == std::vector<radix::tile::Id>{radix::tile::Id(1, {0, 0}), radix::tile::Id(9, {0, 0})});
}

TEST_CASE("texture assembler reports the content region", "[texture_assembler]") {
    // Row gradient, so every row of the assembled image can be traced back to its source row.
    cv::Mat tile_image(256, 256, CV_8UC1);
    for (uint32_t row = 0; row < 256; row++) {
        tile_image.row(row).setTo(uint8_t(row));
    }

    const radix::tile::Id root_tile(0, {0, 0});
    const StaticTileProvider tile_provider({{root_tile, tile_image}});
    const std::vector<radix::tile::Id> tiles_to_splatter = {root_tile};

    const radix::tile::SrsBounds root_tile_bounds = gis::webmercator::compute_tile_bounds(root_tile);
    const radix::tile::SrsBounds target_bounds = bounds_of_root_tile_pixels(root_tile_bounds, glm::uvec2(100, 70), glm::uvec2(150, 160));

    const sf::AssembledTexture assembled = REQUIRE_VALUE(sf::splatter_tiles_to_texture(
        root_tile,
        target_bounds,
        tile_provider,
        tiles_to_splatter,
        cv::INTER_NEAREST_EXACT));

    // Region (96, 80) to (160, 192), with the content region flipped along with the image.
    REQUIRE(assembled.image.size() == cv::Size(64, 112));
    CHECK(assembled.content.min == glm::uvec2(4, 6));
    CHECK(assembled.content.max == glm::uvec2(54, 96));

    // The content rows are the southernmost and northernmost requested rows of the source tile.
    CHECK(assembled.image.at<uint8_t>(assembled.content.min.y, assembled.content.min.x) == uint8_t(185));
    CHECK(assembled.image.at<uint8_t>(assembled.content.max.y - 1, assembled.content.min.x) == uint8_t(96));
}

TEST_CASE("texture assembler fills the region of a tile 24 levels below the root with that tile", "[texture_assembler]") {
    const cv::Mat tile_image(256, 256, CV_8UC1, cv::Scalar(7));
    const radix::tile::Id root_tile(0, {0, 0});
    const radix::tile::Id tile(24, {1u << 23, 1u << 23});
    const StaticTileProvider tile_provider({{tile, tile_image}});
    const std::vector<radix::tile::Id> tiles_to_splatter = {tile};

    // Shrunk by a quarter pixel, so rounding to the pixel grid lands on the tile's edges.
    const radix::tile::SrsBounds tile_bounds = gis::webmercator::compute_tile_bounds(tile);
    const glm::dvec2 quarter_pixel = tile_bounds.size() / 1024.0;
    const radix::tile::SrsBounds target_bounds(tile_bounds.min + quarter_pixel, tile_bounds.max - quarter_pixel);

    const sf::AssembledTexture assembled = REQUIRE_VALUE(sf::splatter_tiles_to_texture(
        root_tile,
        target_bounds,
        tile_provider,
        tiles_to_splatter,
        cv::INTER_NEAREST_EXACT,
        glm::uvec2(0),
        1));

    REQUIRE(assembled.image.size() == cv::Size(256, 256));
    CHECK(cv::countNonZero(assembled.image != 7) == 0);
}

TEST_CASE("assembled texture remaps uvs into the padded image", "[texture_assembler]") {
    const sf::AssembledTexture assembled{
        .image = cv::Mat::zeros(100, 100, CV_8UC1),
        .content = radix::geometry::Aabb2ui(glm::uvec2(10, 20), glm::uvec2(60, 70))};

    std::vector<glm::dvec2> uvs = {glm::dvec2(0, 0), glm::dvec2(1, 1), glm::dvec2(0.5, 0.5)};
    assembled.remap_uvs(uvs);

    CHECK(uvs[0] == glm::dvec2(0.1, 0.2));
    CHECK(uvs[1] == glm::dvec2(0.6, 0.7));
    CHECK(uvs[2] == glm::dvec2(0.35, 0.45));
}

TEST_CASE("Assembling a texture fails when a tile's file is not an image", "[texture_assembler]") {
    const TempDirectory directory("texture_assembler_not_an_image");
    const GoogleMapboxTilePathProvider tile_provider(directory.path());
    const radix::tile::Id root_tile(0, {0, 0});
    REQUIRE_VALUE(io::bytes::write(std::vector<uint8_t>{1, 2, 3, 4}, tile_provider.get_tile_path(root_tile).value()));
    const std::vector<radix::tile::Id> tiles_to_splatter = {root_tile};

    const TileError error = REQUIRE_ERROR(sf::splatter_tiles_to_texture(
        root_tile,
        gis::webmercator::compute_tile_bounds(root_tile),
        tile_provider,
        tiles_to_splatter));

    CHECK(error.holds_code(UnreadableTile{root_tile}));
}

TEST_CASE("Assembling a texture yields none when the provider has no tiles", "[texture_assembler]") {
    const EmptyTileProvider tile_provider;
    const radix::tile::Id tile(3, {5, 4});

    const std::optional<sf::AssembledTexture> texture = REQUIRE_VALUE(sf::assemble_texture_from_tiles(
        gis::webmercator::compute_tile_bounds(tile),
        tile_provider));

    CHECK(!texture.has_value());
}

TEST_CASE("Assembling a texture yields none for bounds outside the tile grid", "[texture_assembler]") {
    const radix::tile::Id root_tile(0, {0, 0});
    const StaticTileProvider tile_provider({{root_tile, cv::Mat(1, 1, CV_8UC3, cv::Vec3b(0, 0, 255))}});
    const radix::tile::SrsBounds grid_bounds = gis::webmercator::compute_tile_bounds(root_tile);
    const radix::tile::SrsBounds outside(grid_bounds.max + glm::dvec2(1, 1), grid_bounds.max + glm::dvec2(101, 101));

    const std::optional<sf::AssembledTexture> texture = REQUIRE_VALUE(sf::assemble_texture_from_tiles(
        outside,
        tile_provider));

    CHECK(!texture.has_value());
}
