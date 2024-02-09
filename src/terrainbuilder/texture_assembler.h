#ifndef TEXTUREASSEMBLER_H
#define TEXTUREASSEMBLER_H

#include <chrono>
#include <filesystem>
#include <numeric>
#include <span>
#include <vector>

#include <fmt/core.h>
#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>
#include <radix/geometry.h>

#include "ctb/GlobalMercator.hpp"
#include "ctb/Grid.hpp"
#include "srs.h"

#include "tile_provider.h"

/// Estimates the zoom level of the target bounds in relation to some reference tile given by its zoom level and bounds.
[[nodiscard]] unsigned int estimate_zoom_level(
    const unsigned int reference_zoom_level,
    const tile::SrsBounds reference_tile_bounds,
    const tile::SrsBounds target_bounds) {
    const glm::dvec2 relative_size = reference_tile_bounds.size() / target_bounds.size();
    const double relative_factor = (relative_size.x + relative_size.y) / 2;
    const int zoom_level_change = std::rint(std::log2(relative_factor));
    assert(static_cast<int>(reference_zoom_level) + zoom_level_change >= 0);
    return reference_zoom_level + zoom_level_change;
}

/// Find all the tiles needed to construct the texture for the given target bounds under the root tile.
/// The result tiles are ordered in such a way that they can be sequentially written to a texture.
/// Larger tiles are only included if they are not covered by smaller tiles.
[[nodiscard]] std::vector<tile::Id> find_relevant_tiles_to_splatter_in_bounds(
    /// The root tile specifying the tiles to consider.
    const tile::Id root_tile,
    /// Specifes the grid used to organize the image tiles.
    const ctb::Grid &grid,
    /// The bounds for which texture data should be created.
    const tile::SrsBounds &target_bounds,
    /// Provider class that maps tile ids to their textures.
    const TileProvider &tile_provider,
    /// The maximal zoom level to be considered.
    const std::optional<unsigned int> max_zoom_level_to_consider = std::nullopt,
    /// The minimum zoom level to be surely examined.
    std::optional<unsigned int> min_zoom_level_to_examine = std::nullopt) {
    // It can happen that the root tile is very large because the target tile was (slightly) over the tile border
    // at a high zoom level. So we try to estimate the actual zoom level of the target bounds and recurse at least
    // to that level to find relevant textures.
    if (!min_zoom_level_to_examine.has_value()) {
        min_zoom_level_to_examine = estimate_zoom_level(root_tile.zoom_level, grid.srsBounds(root_tile, false), target_bounds) + 1;
    }
    
    // A list of tiles determined to be relevant for the target bounds.
    std::vector<tile::Id> tiles_to_splatter;

    const std::function<bool(const tile::Id tile)> helper = [&](const tile::Id tile) {
        // Check if we have recursed too deep.
        if (max_zoom_level_to_consider.has_value() && tile.zoom_level > max_zoom_level_to_consider.value()) {
            return false;
        }

        // Check if we even require this tile to fill the target region.
        const tile::SrsBounds tile_bounds = grid.srsBounds(tile, false);
        if (!geometry::intersect(target_bounds, tile_bounds)) {
            return true;
        }

        // Check the region of this tile can be assembled from its children (or further down)
        bool all_children_present = false;
        if (tile.zoom_level + 1 <= min_zoom_level_to_examine.value()) {
            all_children_present = true;
            for (const tile::Id subtile : tile.children()) {
                if (!helper(subtile)) {
                    all_children_present = false;
                }
            }
        }

        if (all_children_present) {
            return true;
        }

        // Check if the current tile is available as a replacement.
        if (!tile_provider.has_tile(tile)) {
            return false;
        }

        // If so, add it to the list of accepted tiles.
        tiles_to_splatter.push_back(tile);
        return true;
    };

    helper(root_tile);

    std::reverse(tiles_to_splatter.begin(), tiles_to_splatter.end());

    return tiles_to_splatter;
}

/// Calculate the offset and size of the target bounds inside the root tile.
[[nodiscard]] geometry::Aabb2ui calculate_target_image_region(
    /// The bounds for which texture data should be created.
    const tile::SrsBounds target_bounds,
    /// The bounds of the root tile.
    const tile::SrsBounds root_tile_bounds,
    /// The image size of each tile.
    const glm::uvec2 tile_image_pixel_size,
    /// The range of zoom levels from the root to the maximum zoom.
    const unsigned int zoom_level_range) {
    const glm::dvec2 relative_min = (target_bounds.min - root_tile_bounds.min) / root_tile_bounds.size();
    const glm::dvec2 relative_max = (target_bounds.max - root_tile_bounds.min) / root_tile_bounds.size();
    const unsigned int full_image_size_factor = std::pow(2, zoom_level_range);
    const glm::uvec2 root_tile_image_size = tile_image_pixel_size * glm::uvec2(full_image_size_factor);
    const glm::uvec2 target_pixel_offset_min(glm::floor(relative_min * glm::dvec2(root_tile_image_size)));
    const glm::uvec2 target_pixel_offset_max(glm::ceil(relative_max * glm::dvec2(root_tile_image_size)));
    const geometry::Aabb2ui target_image_region(
        glm::uvec2(target_pixel_offset_min.x, root_tile_image_size.y - target_pixel_offset_max.y),
        glm::uvec2(target_pixel_offset_max.x, root_tile_image_size.y - target_pixel_offset_min.y));
    return target_image_region;
}

[[nodiscard]] geometry::Aabb2ui calculate_pixel_tile_bounds(
    const tile::Id tile,
    const tile::Id root_tile,
    const glm::uvec2 tile_image_pixel_size,
    const unsigned int max_zoom_level) {
    const size_t relative_zoom_level = tile.zoom_level - root_tile.zoom_level;
    const glm::uvec2 tile_size_factor = glm::uvec2(std::pow(2, max_zoom_level - tile.zoom_level));
    const glm::uvec2 tile_size = tile_image_pixel_size * tile_size_factor;
    const glm::uvec2 relative_tile_coords = tile.coords - root_tile.coords * glm::uvec2(std::pow(2, relative_zoom_level));
    const glm::uvec2 tile_position = relative_tile_coords * tile_size;
    return geometry::Aabb2ui(tile_position, tile_position + tile_size);
}

void copy_paste_image(
    cv::Mat &target,
    const cv::Mat &source,
    const geometry::Aabb2i target_bounds,
    const bool trim_excess = false,
    const cv::InterpolationFlags rescale_filter = cv::INTER_LINEAR) {
    if (target_bounds.width() != source.cols || target_bounds.height() != source.rows) {
        cv::Mat source_resized;
        cv::resize(source, source, cv::Size(target_bounds.width(), target_bounds.height()), 0, 0, rescale_filter);
        copy_paste_image(target, source_resized, target_bounds, trim_excess);
        return;
    }

    cv::Rect source_rect(0, 0, source.cols, source.rows);
    cv::Rect target_rect(target_bounds.min.x, target_bounds.min.y, target_bounds.width(), target_bounds.height());
    if (trim_excess) {
        const cv::Rect full_target_rect(0, 0, target.cols, target.rows);
        target_rect = target_rect & full_target_rect;
        source_rect = cv::Rect(target_rect.x - target_bounds.min.x, target_rect.y - target_bounds.min.y, target_rect.width, target_rect.height);
    }

    if (source_rect.empty()) {
        return;
    }

    source(source_rect).copyTo(target(target_rect));
}

void copy_paste_image(
    cv::Mat &target,
    const cv::Mat &source,
    const glm::ivec2 target_position,
    const bool trim_excess = false,
    const cv::InterpolationFlags rescale_filter = cv::INTER_LINEAR) {
    copy_paste_image(target, source, geometry::Aabb2i(target_position, target_position + glm::ivec2(source.cols, source.rows)), trim_excess, rescale_filter);
}

[[nodiscard]] cv::Mat splatter_tiles_to_texture(
    const tile::Id root_tile,
    /// Specifes the grid used to organize the image tiles.
    const ctb::Grid &grid,
    /// The bounds for which texture data should be created.
    const tile::SrsBounds &target_bounds,
    /// A mapping from tile id to a filesystem path.
    const TileProvider &tile_provider,
    const std::span<const tile::Id> tiles_to_splatter,
    const cv::InterpolationFlags rescale_filter = cv::INTER_LINEAR) {
    assert(!tiles_to_splatter.empty());
    const tile::SrsBounds root_tile_bounds = grid.srsBounds(root_tile, false);

    unsigned int max_zoom_level = 0;
    for (const tile::Id &tile : tiles_to_splatter) {
        max_zoom_level = std::max(tile.zoom_level, max_zoom_level);
    }
    assert(max_zoom_level >= root_tile.zoom_level);
    const unsigned int zoom_level_range = max_zoom_level - root_tile.zoom_level;

    // Choose any tile to load infer like tile size and format to allocate our texture buffer accordingly.
    const tile::Id &any_tile = tiles_to_splatter.front();
    const cv::Mat any_tile_image = tile_provider.get_tile(any_tile).value();
    const glm::uvec2 tile_image_size(any_tile_image.cols, any_tile_image.rows);

    // Calculate the offset and size of the target bounds inside the smallest encompassing tile.
    // As we dont want to allocate and fill a larger buffer than we have to.
    const geometry::Aabb2ui target_image_region = calculate_target_image_region(target_bounds, root_tile_bounds, tile_image_size, zoom_level_range);
    const glm::uvec2 image_size = target_image_region.size();

    // Allocate the image to write all the individual tiles into.
    cv::Mat image(image_size.y, image_size.x, any_tile_image.type());

    for (const tile::Id &tile : tiles_to_splatter) {
        cv::Mat tile_image = tile_provider.get_tile(tile).value();
        if (tile_image.empty()) {
            const TilePathProvider *tile_path_provider = dynamic_cast<const TilePathProvider *>(&tile_provider);
            if (tile_path_provider != nullptr) {
                const std::filesystem::path tile_path = tile_path_provider->get_tile_path(tile).value();
                throw std::runtime_error(fmt::format("failed to load image from path: {}", tile_path.string()));
            } else {
                throw std::runtime_error("failed to load image");
            }
        }

        const glm::uvec2 current_tile_image_size(tile_image.cols, tile_image.rows);
        if (current_tile_image_size != tile_image_size) {
            throw std::runtime_error{"tiles have inconsistent sizes"};
        }

        // Pixel bounds of this image relative to the root tile.
        const geometry::Aabb2ui pixel_tile_bounds = calculate_pixel_tile_bounds(tile, root_tile, tile_image_size, max_zoom_level);
        assert(glm::all(glm::greaterThanEqual(pixel_tile_bounds.size(), current_tile_image_size)));

        // Pixel bounds relative to the target image texture region.
        const glm::ivec2 tile_target_position = glm::ivec2(pixel_tile_bounds.min) - glm::ivec2(target_image_region.min);
        const geometry::Aabb2i target_pixel_tile_bounds(tile_target_position, tile_target_position + glm::ivec2(pixel_tile_bounds.size()));

        // Resize current tile image and copy into image buffer.
        copy_paste_image(image, tile_image, tile_target_position, true /* allows und handles overflow */, rescale_filter);
    }

    cv::flip(image, image, 0);

    return image;
}

/// Creates a texture for the given region.
[[nodiscard]] std::optional<cv::Mat> assemble_texture_from_tiles(
    /// Specifes the grid used to organize the image tiles.
    const ctb::Grid &grid,
    /// Specifies the srs the target bounds are in.
    const OGRSpatialReference &target_srs,
    /// The bounds for which texture data should be created.
    const tile::SrsBounds &target_bounds,
    /// Provider class that maps tile ids to their textures.
    const TileProvider &tile_provider,
    /// The maximal zoom level to be considered. If not present, this function will use the maximal available.
    const std::optional<unsigned int> max_zoom = std::nullopt,
    /// The filter used to rescale the tile images if required due to missing detail tiles.
    const cv::InterpolationFlags rescale_filter = cv::INTER_LINEAR) {
    if (target_bounds.width() == 0 || target_bounds.height() == 0) {
        return std::nullopt;
    }

    // Start by transforming the input bounds into the srs the tiles are in.
    const tile::SrsBounds encompassing_bounds = srs::encompassing_bounding_box_transfer(target_srs, grid.getSRS(), target_bounds);
    // Then we find the smallest tile (id) that encompasses these bounds.
    const tile::Id smallest_encompassing_tile = grid.findSmallestEncompassingTile(encompassing_bounds).value().to(tile::Scheme::SlippyMap);

    if (max_zoom.has_value() && smallest_encompassing_tile.zoom_level > max_zoom.value()) {
        return std::nullopt;
    }

    // Find relevant tiles in bounds
    const std::vector<tile::Id> tiles_to_splatter = find_relevant_tiles_to_splatter_in_bounds(
        smallest_encompassing_tile, grid, encompassing_bounds, tile_provider, max_zoom);

    // If we found to relevant tiles, we are done.
    if (tiles_to_splatter.empty()) {
        return std::nullopt;
    }

    // Splatter tiles into texture buffer
    return splatter_tiles_to_texture(smallest_encompassing_tile, grid, encompassing_bounds, tile_provider, tiles_to_splatter, rescale_filter);
}

#endif
