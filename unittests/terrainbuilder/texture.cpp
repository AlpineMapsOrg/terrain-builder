/*****************************************************************************
 * Alpine Terrain Builder
 * Copyright (C) 2022 Adam Celarek <last name at cg tuwien ac at>
 * Copyright (C) 2022 alpinemaps.org
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
#include <filesystem>
#include <optional>
#include <ranges>

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <fmt/core.h>

#include "../catch2_helpers.h"
#include "Dataset.h"
#include "ctb/GlobalMercator.hpp"
#include "ctb/Grid.hpp"
#include "srs.h"

#include "mesh/terrain_mesh.h"
#include "texture_assembler.h"

TEST_CASE("estimate_zoom_level", "[terrainbuilder]") {
    const ctb::Grid grid = ctb::GlobalMercator();
    const tile::Id tile(20, glm::uvec2(0, 1), tile::Scheme::SlippyMap);
    const tile::SrsBounds tile_bounds = grid.srsBounds(tile, false);
    const tile::SrsBounds shifted_bounds(tile_bounds.min + glm::dvec2(-100, 420), tile_bounds.max + glm::dvec2(-100, 420));

    REQUIRE(estimate_zoom_level(tile.zoom_level, tile_bounds, shifted_bounds) == tile.zoom_level);
}

class AvailabilityListEmptyTileProvider : public TileProvider {
public:
    AvailabilityListEmptyTileProvider(std::set<tile::Id> available_tiles)
        : available_tiles(available_tiles) {
    }

    virtual std::optional<cv::Mat> get_tile(const tile::Id tile) const override {
        if (this->available_tiles.find(tile) != this->available_tiles.end()) {
            return cv::Mat();
        } else {
            return std::nullopt;
        }
    }

private:
    const std::set<tile::Id> available_tiles;
};

class AlwaysEmptyTileProvider : public TileProvider {
public:
    virtual std::optional<cv::Mat> get_tile(const tile::Id) const override {
        return cv::Mat();
    }
};

TEST_CASE("texture assembler takes root tile if only available ", "[terrainbuilder]") {
    const tile::Id root_tile(3, glm::uvec2(5, 4), tile::Scheme::SlippyMap);
    const std::set<tile::Id> available_tiles = {
        {3, {5, 4}, tile::Scheme::SlippyMap}};
    const std::set<tile::Id> expected_tiles = {
        {3, {5, 4}, tile::Scheme::SlippyMap}};

    const AvailabilityListEmptyTileProvider tile_provider(available_tiles);

    const ctb::Grid grid = ctb::GlobalMercator();
    std::vector<tile::Id> actual_tiles_vec = find_relevant_tiles_to_splatter_in_bounds(
        root_tile,
        grid,
        grid.srsBounds(root_tile, false),
        tile_provider);
    std::set<tile::Id> actual_tiles(std::make_move_iterator(actual_tiles_vec.begin()),
                                    std::make_move_iterator(actual_tiles_vec.end()));

    CHECK(expected_tiles == actual_tiles);
}

TEST_CASE("texture assembler ignores parent if all children are present", "[terrainbuilder]") {
    const tile::Id root_tile(3, glm::uvec2(5, 4), tile::Scheme::SlippyMap);
    const std::set<tile::Id> available_tiles = {
        {3, {5, 4}, tile::Scheme::SlippyMap},
        {4, {10, 8}, tile::Scheme::SlippyMap},
        {4, {11, 8}, tile::Scheme::SlippyMap},
        {4, {10, 9}, tile::Scheme::SlippyMap},
        {4, {11, 9}, tile::Scheme::SlippyMap}};
    const std::set<tile::Id> expected_tiles = {
        {4, {10, 8}, tile::Scheme::SlippyMap},
        {4, {11, 8}, tile::Scheme::SlippyMap},
        {4, {10, 9}, tile::Scheme::SlippyMap},
        {4, {11, 9}, tile::Scheme::SlippyMap}};

    const AvailabilityListEmptyTileProvider tile_provider(available_tiles);

    const ctb::Grid grid = ctb::GlobalMercator();
    std::vector<tile::Id> actual_tiles_vec = find_relevant_tiles_to_splatter_in_bounds(
        root_tile,
        grid,
        grid.srsBounds(root_tile, false),
        tile_provider);
    std::set<tile::Id> actual_tiles(std::make_move_iterator(actual_tiles_vec.begin()),
                                    std::make_move_iterator(actual_tiles_vec.end()));

    CHECK(expected_tiles == actual_tiles);
}

TEST_CASE("texture assembler considers max zoom level", "[terrainbuilder]") {
    const tile::Id root_tile(3, glm::uvec2(5, 4), tile::Scheme::SlippyMap);
    const std::set<tile::Id> available_tiles = {
        {3, {5, 4}, tile::Scheme::SlippyMap},
        {4, {10, 8}, tile::Scheme::SlippyMap},
        {4, {11, 8}, tile::Scheme::SlippyMap},
        {4, {10, 9}, tile::Scheme::SlippyMap},
        {4, {11, 9}, tile::Scheme::SlippyMap}};
    const std::set<tile::Id> expected_tiles = {
        {3, {5, 4}, tile::Scheme::SlippyMap}};

    const AvailabilityListEmptyTileProvider tile_provider(available_tiles);

    const ctb::Grid grid = ctb::GlobalMercator();
    std::vector<tile::Id> actual_tiles_vec = find_relevant_tiles_to_splatter_in_bounds(
        root_tile,
        grid,
        grid.srsBounds(root_tile, false),
        tile_provider,
        3);
    std::set<tile::Id> actual_tiles(std::make_move_iterator(actual_tiles_vec.begin()),
                                    std::make_move_iterator(actual_tiles_vec.end()));

    CHECK(expected_tiles == actual_tiles);
}

TEST_CASE("texture assembler works for arbitrary bounds", "[terrainbuilder]") {
    const std::set<tile::Id> available_tiles = {
        // {21, {1048576, 1048576}, tile::Scheme::Tms},
        {21, {1048577, 1048576}, tile::Scheme::Tms},
        {21, {1048578, 1048576}, tile::Scheme::Tms},
        {21, {1048579, 1048576}, tile::Scheme::Tms},
        {21, {1048580, 1048576}, tile::Scheme::Tms},
        // {21, {1048581, 1048576}, tile::Scheme::Tms},
        {20, {524288, 524288}, tile::Scheme::Tms},
        {20, {524289, 524288}, tile::Scheme::Tms},
        {20, {524290, 524288}, tile::Scheme::Tms},
        // {19, {262144, 262144}, tile::Scheme::Tms},
        {19, {262145, 262144}, tile::Scheme::Tms}};
    const std::set<tile::Id> expected_tiles = {
        // {21, {1048576, 1048576}, tile::Scheme::Tms},
        {21, {1048577, 1048576}, tile::Scheme::Tms},
        {21, {1048578, 1048576}, tile::Scheme::Tms},
        {21, {1048579, 1048576}, tile::Scheme::Tms},
        {21, {1048580, 1048576}, tile::Scheme::Tms},
        // {21, {1048581, 1048576}, tile::Scheme::Tms},
        {20, {524288, 524288}, tile::Scheme::Tms},
        // {20, {524289, 524288}, tile::Scheme::Tms},
        {20, {524290, 524288}, tile::Scheme::Tms},
        // {19, {262144, 262144}, tile::Scheme::Tms},
        // {19, {262145, 262144}, tile::Scheme::Tms}
    };

    const AvailabilityListEmptyTileProvider tile_provider(available_tiles);

    const ctb::Grid grid = ctb::GlobalMercator();
    const tile::SrsBounds target_bounds(glm::dvec2(0, 0), glm::dvec2(100, 1));
    const tile::Id root_tile = grid.findSmallestEncompassingTile(target_bounds).value();
    std::vector<tile::Id> actual_tiles_vec = find_relevant_tiles_to_splatter_in_bounds(
        root_tile,
        grid,
        target_bounds,
        tile_provider,
        23,
        21);
    std::set<tile::Id> actual_tiles(std::make_move_iterator(actual_tiles_vec.begin()),
                                    std::make_move_iterator(actual_tiles_vec.end()));

    CHECK(expected_tiles == actual_tiles);
}

TEST_CASE("texture assembler does not fail if there are not tiles", "[terrainbuilder]") {
    const std::set<tile::Id> available_tiles = {};
    const std::set<tile::Id> expected_tiles = {};

    const AvailabilityListEmptyTileProvider tile_provider(available_tiles);

    const ctb::Grid grid = ctb::GlobalMercator();
    const tile::SrsBounds target_bounds(glm::dvec2(0, 0), glm::dvec2(100, 1));
    const tile::Id root_tile = grid.findSmallestEncompassingTile(target_bounds).value();
    std::vector<tile::Id> actual_tiles_vec = find_relevant_tiles_to_splatter_in_bounds(
        root_tile,
        grid,
        target_bounds,
        tile_provider);
    std::set<tile::Id> actual_tiles(std::make_move_iterator(actual_tiles_vec.begin()),
                                    std::make_move_iterator(actual_tiles_vec.end()));

    CHECK(expected_tiles == actual_tiles);
}

TEST_CASE("texture assembler assembles single tile", "[terrainbuilder]") {
    const std::unordered_map<tile::Id, cv::Mat, tile::Id::Hasher> tiles_to_texture = {
        {tile::Id(0, {0, 0}, tile::Scheme::SlippyMap), cv::Mat(1, 1, CV_8UC3, cv::Vec3b(0, 0, 255))},
    };
    std::vector<tile::Id> tiles_to_splatter;
    std::transform(tiles_to_texture.begin(), tiles_to_texture.end(), std::back_inserter(tiles_to_splatter),
                   [](const auto &pair) { return pair.first; });

    const StaticTileProvider tile_provider(tiles_to_texture);

    const ctb::Grid grid = ctb::GlobalMercator();
    const tile::Id root_tile(0, {0, 0}, tile::Scheme::SlippyMap);
    cv::Mat assembled_texture = splatter_tiles_to_texture(
        root_tile,
        grid,
        grid.srsBounds(root_tile, false),
        tile_provider,
        tiles_to_splatter,
        cv::INTER_NEAREST_EXACT);

    REQUIRE(assembled_texture.size() == cv::Size(1, 1));
    CHECK(assembled_texture.type() == CV_8UC3);

    CHECK(assembled_texture.at<cv::Vec3b>(0, 0) == cv::Vec3b(0, 0, 255));
}

TEST_CASE("texture assembler assembles two tiles", "[terrainbuilder]") {
    const std::unordered_map<tile::Id, cv::Mat, tile::Id::Hasher> tiles_to_texture = {
        {tile::Id(1, {0, 0}, tile::Scheme::SlippyMap), cv::Mat(1, 1, CV_8UC3, cv::Vec3b(0, 0, 255))},
        {tile::Id(1, {0, 1}, tile::Scheme::SlippyMap), cv::Mat(1, 1, CV_8UC3, cv::Vec3b(0, 255, 0))},
    };
    std::vector<tile::Id> tiles_to_splatter;
    std::transform(tiles_to_texture.begin(), tiles_to_texture.end(), std::back_inserter(tiles_to_splatter),
                   [](const auto &pair) { return pair.first; });

    const StaticTileProvider tile_provider(tiles_to_texture);

    const ctb::Grid grid = ctb::GlobalMercator();
    const tile::Id root_tile(0, {0, 0}, tile::Scheme::SlippyMap);
    cv::Mat assembled_texture = splatter_tiles_to_texture(
        root_tile,
        grid,
        grid.srsBounds(root_tile, false),
        tile_provider,
        tiles_to_splatter,
        cv::INTER_NEAREST_EXACT);

    REQUIRE(assembled_texture.size() == cv::Size(2, 2));
    CHECK(assembled_texture.type() == CV_8UC3);

    CHECK(assembled_texture.at<cv::Vec3b>(0, 0) == cv::Vec3b(0, 255, 0));
    CHECK(assembled_texture.at<cv::Vec3b>(1, 0) == cv::Vec3b(0, 0, 255));
}

TEST_CASE("texture assembler correct order of texture writes", "[terrainbuilder]") {
    const std::unordered_map<tile::Id, cv::Mat, tile::Id::Hasher> tiles_to_texture = {
        {tile::Id(0, {0, 0}, tile::Scheme::SlippyMap), cv::Mat(1, 1, CV_8UC1, uint8_t(1))},
        {tile::Id(1, {0, 1}, tile::Scheme::Tms), cv::Mat(1, 1, CV_8UC1, uint8_t(2))},
        {tile::Id(1, {0, 1}, tile::Scheme::SlippyMap), cv::Mat(1, 1, CV_8UC1, uint8_t(3))},
    };
    std::vector<tile::Id> tiles_to_splatter;
    std::transform(tiles_to_texture.begin(), tiles_to_texture.end(), std::back_inserter(tiles_to_splatter),
                   [](const auto &pair) { return pair.first; });
    std::sort(tiles_to_splatter.begin(), tiles_to_splatter.end(),
        [](const tile::Id &a, const tile::Id &b) { return a.zoom_level < b.zoom_level; });

    const StaticTileProvider tile_provider(tiles_to_texture);

    const ctb::Grid grid = ctb::GlobalMercator();
    const tile::Id root_tile(0, {0, 0}, tile::Scheme::SlippyMap);
    cv::Mat assembled_texture = splatter_tiles_to_texture(
        root_tile,
        grid,
        grid.srsBounds(root_tile, false),
        tile_provider,
        tiles_to_splatter,
        cv::INTER_NEAREST_EXACT);

    REQUIRE(assembled_texture.size() == cv::Size(2, 2));
    CHECK(assembled_texture.type() == CV_8UC1);

    CHECK(assembled_texture.at<uint8_t>(0, 0) == uint8_t(3));
    CHECK(assembled_texture.at<uint8_t>(1, 0) == uint8_t(2));
    CHECK(assembled_texture.at<uint8_t>(0, 1) == uint8_t(1));
    CHECK(assembled_texture.at<uint8_t>(1, 1) == uint8_t(1));
}
