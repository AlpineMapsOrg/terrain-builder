#ifndef TEXTUREASSEMBLER_H
#define TEXTUREASSEMBLER_H

#include <chrono>
#include <filesystem>
#include <numeric>
#include <vector>
#include <span>

#include <FreeImage.h>
#include <fmt/core.h>
#include <glm/glm.hpp>
#include <radix/geometry.h>

#include "ctb/GlobalMercator.hpp"
#include "ctb/Grid.hpp"
#include "srs.h"

#include "fi_image.h"

typedef std::function<std::optional<std::filesystem::path>(tile::Id)> TileToPathMapper;
typedef std::function<std::filesystem::path(tile::Id)> TileToPathMapperChecked;

/// Checks whether the given tile can be used to assemble the tile texture.
[[nodiscard]] bool is_usable_tile(
    tile::Id tile,
    const TileToPathMapper &tile_to_path_mapper) {
    const std::optional<std::filesystem::path> tile_path = tile_to_path_mapper(tile);
    return tile_path.has_value() && !tile_path->empty() && std::filesystem::exists(*tile_path);
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
    /// A mapping from tile id to a filesystem path.
    const TileToPathMapper &tile_to_path_mapper,
    /// The maximal zoom level to be considered. If not present, this function will use the maximal available.
    const std::optional<unsigned int> max_zoom = std::nullopt) {
    if (max_zoom.has_value() && root_tile.zoom_level > max_zoom.value()) {
        return {};
    }

    // A stack for tiles to be considered for relevancy.
    std::vector<tile::Id> tile_stack;
    // A list of tiles determined to be relevant for the target bounds.
    std::vector<tile::Id> tiles_to_splatter;
    tile_stack.emplace_back(root_tile);

    while (!tile_stack.empty()) {
        const tile::Id tile = tile_stack.back();
        tile_stack.pop_back();

        const tile::SrsBounds tile_bounds = grid.srsBounds(tile, false);
        if (!geometry::intersect(target_bounds, tile_bounds)) {
            // If this tile does not intersect our target bounds, we can ignore it.
            continue;
        }

        constexpr size_t subtile_count = 4;
        const std::array<tile::Id, subtile_count> subtiles = tile.children();

        std::array<bool, subtile_count> subtile_usable;
        std::transform(subtiles.begin(), subtiles.end(), subtile_usable.begin(),
                       [&](auto &subtile) { return is_usable_tile(subtile, tile_to_path_mapper); });

        const bool all_usable = std::all_of(subtile_usable.begin(), subtile_usable.end(), [](bool e) { return e; });

        // If not all children are present we have to include the current tile to avoid empty regions.
        if (!all_usable) {
            if (max_zoom.has_value()) {
                // If we are using a max zoom we cannot be sure that the current file even exists
                // so double check it.
                if (is_usable_tile(tile, tile_to_path_mapper)) {
                    tiles_to_splatter.push_back(tile);
                }
            } else {
                tiles_to_splatter.push_back(tile);
            }
        }

        // We recurse for every tile that was present or all the time if we were given a maximum zoom
        // (to allow for missing intermediates)
        for (size_t i = 0; i < subtile_count; i++) {
            if (!max_zoom.has_value() && subtile_usable[i] || max_zoom.has_value() && subtiles[i].zoom_level <= max_zoom.value()) {
                tile_stack.push_back(subtiles[i]);
            }
        }
    }

    // Due to the way the above loop is done, we never check whether the root tile is present,
    // if it is the only one selected. So we do that here.
    if (tiles_to_splatter.size() == 1 && tiles_to_splatter[0] == root_tile) {
        if (!is_usable_tile(root_tile, tile_to_path_mapper)) {
            return {};
        }
    }

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

[[nodiscard]] FiImage splatter_tiles_to_texture(
    const tile::Id root_tile,
    /// Specifes the grid used to organize the image tiles.
    const ctb::Grid &grid,
    /// The bounds for which texture data should be created.
    const tile::SrsBounds &target_bounds,
    /// A mapping from tile id to a filesystem path.
    const TileToPathMapperChecked &tile_to_path_mapper,
    const std::vector<tile::Id> tiles_to_splatter,
    const FREE_IMAGE_FILTER rescale_filter = FILTER_BILINEAR) {
    assert(!tiles_to_splatter.empty());
    const tile::SrsBounds root_tile_bounds = grid.srsBounds(root_tile, false);

    unsigned int max_zoom_level = 0;
    for (const tile::Id &tile : tiles_to_splatter) {
        max_zoom_level = std::max(tile.zoom_level, max_zoom_level);
    }
    const unsigned int zoom_level_range = max_zoom_level - root_tile.zoom_level;

    // Choose any tile to load infer like tile size and format to allocate our texture buffer accordingly.
    const tile::Id &any_tile = tiles_to_splatter.front();
    const std::filesystem::path &any_tile_path = tile_to_path_mapper(any_tile);
    const FiImage any_tile_image = FiImage::load_from_path(any_tile_path);
    const glm::uvec2 tile_size = any_tile_image.size();

    // Calculate the offset and size of the target bounds inside the smallest encompassing tile.
    // As we dont want to allocate and fill a larger buffer than we have to.
    const geometry::Aabb2ui target_image_region = calculate_target_image_region(target_bounds, root_tile_bounds, tile_size, zoom_level_range);
    const glm::uvec2 image_size = target_image_region.size();

    // Allocate the image to write all the individual tiles into.
    FiImage image = FiImage::allocate_like(any_tile_image, target_image_region.size());

    for (const tile::Id &tile : tiles_to_splatter) {
        // We start by calculating the relative position of each tile inside the root tile
        // and then use the previosly calculated target image region to determine the position in the buffer.
        const geometry::Aabb2ui pixel_tile_bounds = calculate_pixel_tile_bounds(tile, root_tile, tile_size, max_zoom_level);

        const std::filesystem::path &tile_path = tile_to_path_mapper(tile);
        FiImage tile_image = FiImage::load_from_path(tile_path);

        if (tile_image.width() != tile_size.x) {
            throw std::runtime_error{"tiles have inconsistent widths"};
        }
        if (tile_image.height() != tile_size.y) {
            throw std::runtime_error{"tiles have inconsistent heights"};
        }

        assert(glm::all(glm::greaterThanEqual(pixel_tile_bounds.size(), tile_image.size())));

        if (tile_image.size() != pixel_tile_bounds.size()) {
            tile_image = tile_image.rescale(pixel_tile_bounds.size(), rescale_filter);
        }

        const glm::ivec2 tile_target_position(glm::ivec2(pixel_tile_bounds.min) - glm::ivec2(target_image_region.min));
        image.paste(tile_image, tile_target_position, true /* allows und handles overflow */);
    }

    image.flip_vertical();

    return image;
}

/// Creates a texture for the given region.
[[nodiscard]] std::optional<FiImage> assemble_texture_from_tiles(
    /// Specifes the grid used to organize the image tiles.
    const ctb::Grid &grid,
    /// Specifies the srs the target bounds are in.
    const OGRSpatialReference &target_srs,
    /// The bounds for which texture data should be created.
    const tile::SrsBounds &target_bounds,
    /// A mapping from tile id to a filesystem path.
    const TileToPathMapper tile_to_path_mapper,
    /// The maximal zoom level to be considered. If not present, this function will use the maximal available.
    const std::optional<unsigned int> max_zoom = std::nullopt,
    /// The filter used to rescale the tile images if required due to missing detail tiles.
    const FREE_IMAGE_FILTER rescale_filter = FILTER_BILINEAR) {
    if (target_bounds.width() == 0 || target_bounds.height() == 0) {
        return std::nullopt;
    }

    // Start by transforming the input bounds into the srs the tiles are in.
    const tile::SrsBounds encompassing_bounds = srs::encompassing_bounding_box_transfer(target_srs, grid.getSRS(), target_bounds);
    // Then we find the smallest tile (id) that encompasses these bounds.
    const tile::Id smallest_encompassing_tile = grid.findSmallestEncompassingTile(encompassing_bounds).to(tile::Scheme::SlippyMap);

    if (max_zoom.has_value() && smallest_encompassing_tile.zoom_level > max_zoom.value()) {
        return std::nullopt;
    }

    // Find relevant tiles in bounds
    const std::vector<tile::Id> tiles_to_splatter = find_relevant_tiles_to_splatter_in_bounds(
        smallest_encompassing_tile, grid, encompassing_bounds, tile_to_path_mapper, max_zoom);

    // If we found to relevant tiles, we are done.
    if (tiles_to_splatter.empty()) {
        return std::nullopt;
    }

    const TileToPathMapperChecked tile_to_path_mapper_checked = [=](tile::Id tile) {
#ifdef DEBUG
        assert(is_tile_usable(tile));
#endif
        const auto tile_path = tile_to_path_mapper(tile);
        assert(tile_path.has_value());
        return tile_path.value();
    };
    return splatter_tiles_to_texture(smallest_encompassing_tile, grid, encompassing_bounds, tile_to_path_mapper_checked, tiles_to_splatter, rescale_filter);
}

#endif
