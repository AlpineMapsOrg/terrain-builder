#ifndef TEXTUREASSEMBLER_H
#define TEXTUREASSEMBLER_H

#include <chrono>
#include <filesystem>
#include <numeric>
#include <vector>

#include <FreeImage.h>
#include <fmt/core.h>
#include <glm/glm.hpp>
#include <radix/geometry.h>

#include "ctb/GlobalMercator.hpp"
#include "ctb/Grid.hpp"
#include "srs.h"

#include "fi_image.h"

/// Creates a texture for the given region.
std::optional<FiImage> assemble_texture_from_tiles(
    /// Specifes the grid used to organize the image tiles.
    const ctb::Grid &grid,
    /// Specifies the srs the target bounds are in.
    const OGRSpatialReference &target_srs,
    /// The bounds for which texture data should be created.
    const tile::SrsBounds &target_bounds,
    /// A mapping from tile id to a filesystem path.
    const std::function<std::filesystem::path(tile::Id)> tile_to_path_mapper,
    /// The maximal zoom level to be considered. If not present, this function will use the maximal available.
    const std::optional<unsigned int> max_zoom = std::nullopt,
    /// The filter used to rescale the tile images if required due to missing detail tiles.
    const FREE_IMAGE_FILTER rescale_filter = FILTER_BILINEAR) {
    if (target_bounds.width() == 0 || target_bounds.height() == 0) {
        return std::nullopt;
    }

    // Start by transforming the input bounds into the srs the tiles are in.
    const tile::SrsBounds encompassing_bounds = srs::encompassing_bounding_box_transfer(target_srs, grid.getSRS(), target_bounds);
    // Then we find the smalles tile (id) that encompasses these bounds.
    const tile::Id smallest_encompassing_tile = grid.findSmallestEncompassingTile(encompassing_bounds).to(tile::Scheme::SlippyMap);
    const tile::SrsBounds smallest_encompassing_tile_bounds = grid.srsBounds(smallest_encompassing_tile, false);


    // ********************* Find relevant tiles in bounds ********************* //

    // A stack for tiles to be considered for relevancy.
    std::vector<tile::Id> tile_stack;
    // A list of tiles determined to be relevant for the target bounds.
    std::vector<tile::Id> tiles_to_splatter;
    tile_stack.emplace_back(smallest_encompassing_tile);

    while (!tile_stack.empty()) {
        const tile::Id tile = tile_stack.back();
        tile_stack.pop_back();

        const tile::SrsBounds tile_bounds = grid.srsBounds(tile, false);
        if (!geometry::intersect(encompassing_bounds, tile_bounds)) {
            // If this tile does not intersect our target bounds, we can ignore it.
            continue;
        }

        constexpr size_t subtile_count = 4;
        const std::array<tile::Id, subtile_count> subtiles = tile.children();
        std::array<bool, subtile_count> subtile_present;
        std::array<std::filesystem::path, subtile_count> subtile_path;

        for (size_t i = 0; i < subtile_count; i++) {
            subtile_path[i] = tile_to_path_mapper(subtiles[i]);
            subtile_present[i] = !subtile_path[i].empty() && std::filesystem::exists(subtile_path[i]);
        }

        bool all_present = std::all_of(subtile_present.begin(), subtile_present.end(), [](bool e) { return e; });

        // If not all children are present we have to include the current tile to avoid empty regions.
        if (!all_present) {
            if (max_zoom.has_value()) {
                // If we are using a max zoom we cannot be sure that the current file even exists
                // so double check it.
                const std::filesystem::path tile_path = tile_to_path_mapper(tile);
                if (std::filesystem::exists(tile_path)) {
                    tiles_to_splatter.push_back(tile);
                }
            } else {
                tiles_to_splatter.push_back(tile);
            }
        }

        // We recurse for every tile that was present or all the time if we were given a maximum zoom
        // (to allow for missing intermediates)
        for (size_t i = 0; i < subtile_count; i++) {
            if (subtile_present[i] || (max_zoom.has_value() && subtiles[i].zoom_level <= max_zoom.value())) {
                tile_stack.push_back(subtiles[i]);
            }
        }
    }

    // If we found to relevant tiles, we are done.
    if (tiles_to_splatter.empty()) {
        return std::nullopt;
    }

    // Due to the way the above loop is done, we never check whether the root tile is present,
    // if it is the only one selected. So we do that here.
    if (tiles_to_splatter.size() == 1 && tiles_to_splatter[0] == smallest_encompassing_tile) {
        const std::filesystem::path root_tile_path = tile_to_path_mapper(smallest_encompassing_tile);
        if (root_tile_path.empty() || !std::filesystem::exists(root_tile_path)) {
            return std::nullopt;
        }
    }

    // ********************* Splatter tiles into texture buffer ********************* //
    unsigned int root_zoom_level = smallest_encompassing_tile.zoom_level;
    unsigned int min_zoom_level = 0;
    unsigned int max_zoom_level = 0;
    for (const tile::Id &tile : tiles_to_splatter) {
        min_zoom_level = std::min(tile.zoom_level, min_zoom_level);
        max_zoom_level = std::max(tile.zoom_level, max_zoom_level);
    }
    const unsigned int zoom_level_range = max_zoom_level - root_zoom_level;

    // Choose any tile to load infor like tile size and format to allocate our texture buffer accordingly.
    const tile::Id &any_tile = tiles_to_splatter.front();
    const std::filesystem::path &any_tile_path = tile_to_path_mapper(any_tile);
    const FiImage any_tile_image = FiImage::load_from_path(any_tile_path);
    const glm::uvec2 tile_size = any_tile_image.size();

    // Calculate the offset and size of the target bounds inside the smalles encompassing tile.
    // As we dont want to allocate and fill a larger buffer than we have to.
    const glm::dvec2 relative_min = (encompassing_bounds.min - smallest_encompassing_tile_bounds.min) / smallest_encompassing_tile_bounds.size();
    const glm::dvec2 relative_max = (encompassing_bounds.max - smallest_encompassing_tile_bounds.min) / smallest_encompassing_tile_bounds.size();
    const size_t full_image_size_factor = std::pow(2, zoom_level_range);
    const glm::uvec2 image_size_for_smallest_encompassing_tile = tile_size * glm::uvec2(full_image_size_factor);
    const glm::uvec2 target_pixel_offset_min(glm::floor(relative_min * glm::dvec2(image_size_for_smallest_encompassing_tile)));
    const glm::uvec2 target_pixel_offset_max(glm::ceil(relative_max * glm::dvec2(image_size_for_smallest_encompassing_tile)));
    // const glm::uvec2 pixel_offset_min(glm::round(relative_min * glm::dvec2(image_size_for_smallest_encompassing_tile)));
    // const glm::uvec2 pixel_offset_max(glm::round(relative_max * glm::dvec2(image_size_for_smallest_encompassing_tile)));
    const geometry::Aabb2ui target_image_region(
        glm::uvec2(target_pixel_offset_min.x, image_size_for_smallest_encompassing_tile.y - target_pixel_offset_max.y),
        glm::uvec2(target_pixel_offset_max.x, image_size_for_smallest_encompassing_tile.y - target_pixel_offset_min.y));
    const glm::uvec2 image_size = target_image_region.size();

    // Allocate the image to write all the individual tiles into.
    FiImage image = FiImage::allocate_like(any_tile_image, image_size);

    for (const tile::Id &tile : tiles_to_splatter) {
        // We start by calculating the relative position of each tile inside the reference tile (smallest encompassing tile)
        // and then use the previosly calculated offset to determine the position in the buffer.
        const size_t relative_zoom_level = tile.zoom_level - root_zoom_level;
        const glm::uvec2 current_tile_size_factor = glm::uvec2(std::pow(2, max_zoom_level - tile.zoom_level));
        const glm::uvec2 current_tile_size = tile_size * current_tile_size_factor;
        const glm::uvec2 relative_tile_coords = tile.coords - smallest_encompassing_tile.coords * glm::uvec2(std::pow(2, relative_zoom_level));
        const glm::uvec2 current_tile_position = relative_tile_coords * current_tile_size;

        const std::filesystem::path &tile_path = tile_to_path_mapper(tile);
        FiImage tile_image = FiImage::load_from_path(tile_path);

        if (tile_image.width() != tile_size.x) {
            throw std::runtime_error{"tiles have inconsistent widths"};
        }
        if (tile_image.height() != tile_size.y) {
            throw std::runtime_error{"tiles have inconsistent heights"};
        }
        tile_image = tile_image.rescale(current_tile_size, FILTER_BICUBIC);

        const glm::ivec2 tile_target_position(glm::ivec2(current_tile_position) - glm::ivec2(target_image_region.min));
        image.paste(tile_image, tile_target_position, true /* allows und handles overflow */);
    }

    image.flip_vertical();

    return image;
}

#endif
