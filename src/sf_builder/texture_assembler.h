/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2023 Martin Braunsperger
 * Copyright (C) 2023 Adam Celarek-Litofcenko
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

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <numeric>
#include <optional>
#include <span>
#include <vector>
#include <cstdint>
#include <csignal>

#include <fmt/core.h>
#include <glm/glm.hpp>
#include <glm/vector_relational.hpp>
#include <glm/gtc/type_precision.hpp>
#include <opencv2/opencv.hpp>
#include <radix/geometry.h>

#include "geometry/geometry.h"
#include "numeric/glm_math.h"
#include "numeric/int_math.h"
#include "numeric/checked_cast.h"
#include "optional_utils.h"
#include "gis/webmercator.h"

#include "error/try.h"
#include "TileProvider.h"
#include "cli_support/log.h"

namespace sf {

/// Estimates the zoom level of the target bounds in relation to some reference tile given by its zoom level and bounds.
[[nodiscard]] inline uint32_t estimate_zoom_level(
    const uint32_t reference_zoom_level,
    const radix::tile::SrsBounds reference_tile_bounds,
    const radix::tile::SrsBounds target_bounds) {
    const glm::dvec2 relative_size = reference_tile_bounds.size() / target_bounds.size();
    const double relative_factor = (relative_size.x + relative_size.y) / 2;
    const int32_t zoom_level_change = std::rint(std::log2(relative_factor));
    DEBUG_ASSERT(static_cast<int32_t>(reference_zoom_level) + zoom_level_change >= 0);
    return reference_zoom_level + zoom_level_change;
}

/// Searches for the smallest Web Mercator tile that encompasses the given bounding box.
[[nodiscard]] inline std::optional<radix::tile::Id> find_smallest_encompassing_tile(const radix::tile::SrsBounds& bounds) {
    const std::array<glm::dvec2, 4> points = { bounds.min, bounds.max, glm::dvec2(bounds.min.x, bounds.max.y), glm::dvec2(bounds.max.x, bounds.min.y) };
    const auto all_points_inside = [&](const radix::tile::Id& tile) {
        const radix::tile::SrsBounds tile_bounds = gis::webmercator::compute_tile_bounds(tile);
        return std::ranges::all_of(points, [&](const glm::dvec2& point) { return tile_bounds.contains_inclusive(point); });
    };

    // We start at the root tile and repeatedly check every subtile until we find one that contains all of the bounding box.
    radix::tile::Id current_smallest_encompassing_tile(0, { 0, 0 });
    if (!all_points_inside(current_smallest_encompassing_tile)) {
        return std::nullopt;
    }

    // We dont want to recurse indefinetely if the bounds are empty.
    while (current_smallest_encompassing_tile.zoom_level < gis::webmercator::max_zoom_level) {
        const std::array<radix::tile::Id, 4> children = current_smallest_encompassing_tile.children();
        const auto child = std::ranges::find_if(children, all_points_inside);
        // Stop if none of the children contains all of the points.
        if (child == children.end()) {
            return current_smallest_encompassing_tile;
        }
        current_smallest_encompassing_tile = *child;
    }

    return current_smallest_encompassing_tile;
}

/// Find all the tiles needed to construct the texture for the given target bounds under the root tile.
/// The result tiles are ordered in such a way that they can be sequentially written to a texture.
/// Larger tiles are only included if they are not covered by smaller tiles.
[[nodiscard]] inline std::vector<radix::tile::Id> find_relevant_tiles_to_splatter_in_bounds(
    /// The root tile specifying the tiles to consider.
    const radix::tile::Id root_tile,
    /// The bounds for which texture data should be created.
    const radix::tile::SrsBounds& target_bounds,
    /// Provider class that maps tile ids to their textures.
    const TileProvider& tile_provider,
    /// The maximal zoom level to be considered.
    const std::optional<uint32_t> max_zoom_level_to_consider = std::nullopt,
    /// The minimum zoom level to be surely examined.
    std::optional<uint32_t> min_zoom_level_to_examine = std::nullopt) {
    // It can happen that the root tile is very large because the target tile was (slightly) over the tile border
    // at a high zoom level. So we try to estimate the actual zoom level of the target bounds and recurse at least
    // to that level to find relevant textures.
    const uint32_t effective_min_zoom_level_to_examine = alp::value_or_else(min_zoom_level_to_examine, [&] {
        return estimate_zoom_level(root_tile.zoom_level, gis::webmercator::compute_tile_bounds(root_tile), target_bounds) + 1;
    });

    // A list of tiles determined to be relevant for the target bounds.
    std::vector<radix::tile::Id> tiles_to_splatter;

    const std::function<bool(const radix::tile::Id tile)> helper = [&](const radix::tile::Id tile) {
        // Check if we have recursed too deep.
        if (max_zoom_level_to_consider.has_value() && tile.zoom_level > max_zoom_level_to_consider.value()) {
            return false;
        }

        // Check if we even require this tile to fill the target region.
        const radix::tile::SrsBounds tile_bounds = gis::webmercator::compute_tile_bounds(tile);
        if (!radix::geometry::intersect_inclusive(target_bounds, tile_bounds)) {
            return true;
        }

        // Check if the current tile is available.
        const bool has_current_tile = tile_provider.has_tile(tile);

        // If we are not deep enough yet, recurse further down.
        bool should_recurse;
        if (tile.zoom_level + 1 <= effective_min_zoom_level_to_examine) {
            // Minimum zoom level to examine not yet reached.
            should_recurse = true;
        } else {
            // Minimum zoom level reached, only continue if the current tile is available.
              should_recurse = has_current_tile;
        }

        // Check the region of this tile can be assembled from its children (or further down)
        if (should_recurse) {
            bool all_children_present = true;

            for (const radix::tile::Id subtile : tile.children()) {
                if (!helper(subtile)) {
                    all_children_present = false;
                }
            }

            if (all_children_present) {
                return true;
            }
        }

        // Check if the current tile is available as a replacement.
        if (!has_current_tile) {
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

/// Padding kept around the requested region to avoid bleeding at its borders.
constexpr glm::uvec2 TexturePadding(2);
/// Alignment of the assembled region, matches the JPEG macroblock size.
constexpr uint32_t TextureAlignment = 16;
/// Relative growth of the tile selection bounds, so the padding can be filled from neighbouring tiles.
constexpr double TextureSelectionGrowth = 0.01;
/// Most zoom levels a fallback tile may lie above the deepest tile it is drawn with.
constexpr uint32_t MaxFallbackZoomLevels = 8;

struct TargetImageRegion {
    // Region with padding and alignemnt
    radix::geometry::Aabb2<uint64_t> full;

    // Requested region, relative to `full`.
    radix::geometry::Aabb2ui content;
};

/// Calculate the offset and size of the target bounds inside the root tile.
[[nodiscard]]
inline TargetImageRegion calculate_target_image_region(
    /// The bounds for which texture data should be created.
    const radix::tile::SrsBounds target_bounds,
    /// The bounds of the root tile.
    const radix::tile::SrsBounds root_tile_bounds,
    /// The image size of each tile.
    const glm::uvec2 tile_image_pixel_size,
    /// The range of zoom levels from the root to the maximum zoom.
    const uint32_t zoom_level_range,
    /// Minimum padding to apply
    const glm::uvec2 min_padding = TexturePadding,
    /// Alignment of the region (to reuse JPEG blocks)
    const uint64_t alignment = TextureAlignment
) {
    DEBUG_ASSERT(alignment >= 1);
    const glm::dvec2 relative_min = (target_bounds.min - root_tile_bounds.min) / root_tile_bounds.size();
    const glm::dvec2 relative_max = (target_bounds.max - root_tile_bounds.min) / root_tile_bounds.size();

    const uint64_t full_image_size_factor = ipow2<uint64_t>(zoom_level_range);
    const glm::u64vec2 root_tile_image_size = glm::u64vec2(tile_image_pixel_size) * full_image_size_factor;
    const glm::u64vec2 target_pixel_offset_min(glm::floor(relative_min * glm::dvec2(root_tile_image_size)));
    const glm::u64vec2 target_pixel_offset_max(glm::ceil(relative_max * glm::dvec2(root_tile_image_size)));

    // Map from bottom-left origin to top-left origin
    const radix::geometry::Aabb2<uint64_t> requested(
        glm::u64vec2(target_pixel_offset_min.x, root_tile_image_size.y - target_pixel_offset_max.y),
        glm::u64vec2(target_pixel_offset_max.x, root_tile_image_size.y - target_pixel_offset_min.y));

    // Apply padding
    const radix::geometry::Aabb2<uint64_t> padded(
        saturating_sub(requested.min, glm::u64vec2(min_padding)),
        saturating_add(requested.max, glm::u64vec2(min_padding)));

    // Apply alignment
    const radix::geometry::Aabb2<uint64_t> aligned(
        glm::u64vec2(align_down(padded.min.x, alignment), align_down(padded.min.y, alignment)),
        glm::u64vec2(align_up(padded.max.x, alignment), align_up(padded.max.y, alignment)));

    // Clamp to the root tile image, alignment is best effort at its borders
    const radix::geometry::Aabb2<uint64_t> clamped(
        aligned.min,
        glm::min(aligned.max, root_tile_image_size));

    // Compute relative requested
    const radix::geometry::Aabb2ui content = {
        glm::uvec2(requested.min - clamped.min),
        glm::uvec2(requested.max - clamped.min)};

    return TargetImageRegion{
        .full = clamped,
        .content = content};
}

[[nodiscard]] inline radix::geometry::Aabb2<uint64_t> calculate_pixel_tile_bounds(
    radix::tile::Id tile,
    const radix::tile::Id root_tile,
    const glm::uvec2 tile_image_pixel_size,
    const uint32_t max_zoom_level) {
    const size_t relative_zoom_level = tile.zoom_level - root_tile.zoom_level;
    const uint64_t tile_size_factor = ipow2<uint64_t>(max_zoom_level - tile.zoom_level);
    const glm::u64vec2 tile_size = glm::u64vec2(tile_image_pixel_size) * tile_size_factor;
    const glm::u64vec2 relative_tile_coords = glm::u64vec2(tile.coords) - glm::u64vec2(root_tile.coords) * ipow2<uint64_t>(relative_zoom_level);
    const glm::u64vec2 tile_position = relative_tile_coords * tile_size;
    return radix::geometry::Aabb2<uint64_t>(tile_position, tile_position + tile_size);
}

template <typename T>
[[nodiscard]] inline cv::Rect to_cv_rect(radix::geometry::Aabb2<T> aabb) {
    const glm::ivec2 min = debug_checked_cast<int32_t>(aabb.min);
    const glm::ivec2 size = debug_checked_cast<int32_t>(aabb.size());
    return cv::Rect{min.x, min.y, size.x, size.y};
}
[[nodiscard]] inline cv::Size to_cv_size(glm::uvec2 vec) {
    const glm::ivec2 size = debug_checked_cast<int32_t>(vec);
    return cv::Size{size.x, size.y};
}

inline void copy_paste_image(
    cv::Mat& target,
    const cv::Mat& source,
    const radix::geometry::Aabb2i64 target_bounds,
    const bool trim_excess = false,
    const cv::InterpolationFlags rescale_filter = cv::INTER_LINEAR) {
    // Full source/target rectangles
    const cv::Rect full_source_rect(0, 0, source.cols, source.rows);
    const radix::geometry::Aabb2i64 full_target_bounds(glm::i64vec2(0), glm::i64vec2(target.cols, target.rows));

    // Compute scale factors from source to target
    const glm::dvec2 target_size(target_bounds.size());
    const glm::dvec2 scale(
        full_source_rect.width / target_size.x,
        full_source_rect.height / target_size.y);

    // Check bounds
    const radix::geometry::Aabb2i64 clipped_target_bounds(
        glm::max(target_bounds.min, full_target_bounds.min),
        glm::min(target_bounds.max, full_target_bounds.max));
    if (!trim_excess) {
        // Must fit entirely
        ASSERT(clipped_target_bounds == target_bounds);
    } else if (glm::any(glm::lessThanEqual(clipped_target_bounds.max, clipped_target_bounds.min))) {
        return;
    }
    // Target rectangle for source image in target image
    const cv::Rect target_rect = to_cv_rect(clipped_target_bounds);

    // Calculate offset and size of clipped target rect
    const glm::i64vec2 trg_offset = clipped_target_bounds.min - target_bounds.min;
    const glm::uvec2 trg_size = glm::uvec2(target_rect.width, target_rect.height);

    // Exact source region corresponding to clipped target rect
    const radix::geometry::Aabb2d src_exact(glm::dvec2(trg_offset) * scale, glm::dvec2(trg_offset + glm::i64vec2(trg_size)) * scale);

    // Pad outwards to integer pixel boundaries for OpenCV crop
    const radix::geometry::Aabb2ui src_padded(
        glm::uvec2(glm::floor(src_exact.min)),
        glm::uvec2(glm::ceil(src_exact.max)));
    const radix::geometry::Aabb2d src_padding(
        src_exact.min - glm::dvec2(src_padded.min),
        glm::dvec2(src_padded.max) - src_exact.max);
    DEBUG_ASSERT(glm::all(glm::greaterThanEqual(src_padding.min, glm::dvec2(0))));
    DEBUG_ASSERT(glm::all(glm::greaterThanEqual(src_padding.max, glm::dvec2(0))));

    // Safe integer crop of source
    const cv::Rect src_padded_cv = to_cv_rect(src_padded);
    DEBUG_ASSERT(src_padded_cv == (src_padded_cv & full_source_rect));
    const cv::Mat source_cropped = source(src_padded_cv);

    // Compute the target size including padding
    const radix::geometry::Aabb2ui target_padding(
        glm::uvec2(glm::round(src_padding.min / scale)),
        glm::uvec2(glm::round(src_padding.max / scale))
    );
    const glm::uvec2 padded_target_size = target_padding.min + trg_size + target_padding.max;

    // Resize the padded crop -> padded target size
    cv::Mat source_resized;
    cv::resize(source_cropped, source_resized, to_cv_size(padded_target_size), 0, 0, rescale_filter);

    // Crop off the padding
    const radix::geometry::Aabb2ui unpad_rect(
        target_padding.min,
        target_padding.min + trg_size);
    const cv::Rect unpad_rect_cv = to_cv_rect(unpad_rect);
    DEBUG_ASSERT(unpad_rect_cv == (unpad_rect_cv & cv::Rect(0, 0, source_resized.cols, source_resized.rows)));

    const cv::Mat final_source = source_resized(unpad_rect_cv);

    // Copy into target
    final_source.copyTo(target(target_rect));
}

inline void copy_paste_image(
    cv::Mat& target,
    const cv::Mat& source,
    const glm::ivec2 target_position,
    const bool trim_excess = false,
    const cv::InterpolationFlags rescale_filter = cv::INTER_LINEAR) {
    const glm::i64vec2 target_min(target_position);
    const radix::geometry::Aabb2i64 target_bounds(target_min, target_min + glm::i64vec2(source.cols, source.rows));
    copy_paste_image(target, source, target_bounds, trim_excess, rescale_filter);
}

struct AssembledTexture {
    cv::Mat image;

    // Region of `image` covering the requested bounds.
    radix::geometry::Aabb2ui content;

    /// Maps uvs relative to the requested region into the assembled image.
    void remap_uvs(const std::span<glm::dvec2> uvs) const {
        DEBUG_ASSERT(!this->image.empty());
        const glm::dvec2 image_size(this->image.cols, this->image.rows);
        const glm::dvec2 offset = glm::dvec2(this->content.min) / image_size;
        const glm::dvec2 scale = glm::dvec2(this->content.size()) / image_size;
        for (glm::dvec2& uv : uvs) {
            uv = offset + uv * scale;
        }
    }
};

/// Find the deepest zoom level present in the given tiles.
[[nodiscard]]
inline uint32_t find_max_zoom_level(const std::span<const radix::tile::Id> tiles) {
    uint32_t max_zoom_level = 0;
    for (const radix::tile::Id& tile : tiles) {
        max_zoom_level = std::max(tile.zoom_level, max_zoom_level);
    }
    return max_zoom_level;
}

/// Removes every tile more than `max_fallback_zoom_levels` above the deepest one.
inline void drop_coarse_fallback_tiles(std::vector<radix::tile::Id> &tiles, const uint32_t max_fallback_zoom_levels = MaxFallbackZoomLevels) {
    const uint32_t deepest_zoom_level = find_max_zoom_level(tiles);
    std::erase_if(tiles, [&](const radix::tile::Id tile) {
        return deepest_zoom_level - tile.zoom_level > max_fallback_zoom_levels;
    });
}

// Logs the size of a texture about to be allocated, at a higher level the more memory it takes.
inline void log_texture_allocation(const glm::uvec2 image_size, const size_t element_size) {
    const float expected_memory_mb = (element_size * image_size.x * image_size.y) / (1024.0 * 1024.0);
    if (expected_memory_mb > 1000.0f) {
        const float expected_memory_gb = expected_memory_mb / 1024.0f;
        LOG_WARN("Allocating very large texture of size {}x{} (≈{:.2f} GB)", image_size.x, image_size.y, expected_memory_gb);
    } else if (expected_memory_mb > 100.0f) {
        LOG_INFO("Allocating large texture of size {}x{} (≈{:.2f} MB)", image_size.x, image_size.y, expected_memory_mb);
    } else {
        LOG_DEBUG("Allocating texture of size {}x{} (≈{:.2f} MB)", image_size.x, image_size.y, expected_memory_mb);
    }
}

[[nodiscard]]
inline alp::Result<AssembledTexture, TileError> splatter_tiles_to_texture(
    const radix::tile::Id root_tile,
    /// The bounds for which texture data should be created.
    const radix::tile::SrsBounds& target_bounds,
    /// A mapping from tile id to a filesystem path.
    const TileProvider& tile_provider,
    const std::span<const radix::tile::Id> tiles_to_splatter,
    const cv::InterpolationFlags rescale_filter = cv::INTER_LINEAR,
    /// Minimum padding to keep around the requested region.
    const glm::uvec2 min_padding = TexturePadding,
    /// Alignment of the assembled region.
    const uint32_t alignment = TextureAlignment) {
    if (tiles_to_splatter.empty()) {
        return {};
    }

    const radix::tile::SrsBounds root_tile_bounds = gis::webmercator::compute_tile_bounds(root_tile);

    const uint32_t max_zoom_level = find_max_zoom_level(tiles_to_splatter);
    DEBUG_ASSERT(max_zoom_level >= root_tile.zoom_level);
    const uint32_t zoom_level_range = max_zoom_level - root_tile.zoom_level;

    // Choose any tile to infer tile size and format to allocate our texture buffer accordingly.
    const radix::tile::Id& any_tile = tiles_to_splatter.front();
    ALP_TRY(const cv::Mat any_tile_image, tile_provider.get_tile(any_tile));
    const glm::uvec2 tile_image_size(any_tile_image.cols, any_tile_image.rows);

    // Calculate the offset and size of the target bounds inside the smallest encompassing tile.
    // As we dont want to allocate and fill a larger buffer than we have to.
    const TargetImageRegion target_image_region = calculate_target_image_region(target_bounds, root_tile_bounds, tile_image_size, zoom_level_range, min_padding, alignment);
    const glm::uvec2 image_size(target_image_region.full.size());

    // Allocate the image to write all the individual tiles into.
    log_texture_allocation(image_size, any_tile_image.elemSize());
    cv::Mat image = cv::Mat::zeros(image_size.y, image_size.x, any_tile_image.type());

    for (const radix::tile::Id& tile : tiles_to_splatter) {
        ALP_TRY(const cv::Mat tile_image, tile_provider.get_tile(tile));
        DEBUG_ASSERT(!tile_image.empty());

        const glm::uvec2 current_tile_image_size(tile_image.cols, tile_image.rows);
        if (current_tile_image_size != tile_image_size) {
            LOG_WARN("Tiles have inconsistent sizes: tile [{}, ({}, {})] is {}x{} and tile [{}, ({}, {})] is {}x{}",
                tile.zoom_level, tile.coords.x, tile.coords.y,
                current_tile_image_size.x, current_tile_image_size.y,
                any_tile.zoom_level, any_tile.coords.x, any_tile.coords.y,
                tile_image_size.x, tile_image_size.y);
        }

        // Pixel bounds of this image relative to the root tile.
        const radix::geometry::Aabb2<uint64_t> pixel_tile_bounds = calculate_pixel_tile_bounds(tile, root_tile, tile_image_size, max_zoom_level);
        DEBUG_ASSERT(glm::all(glm::greaterThanEqual(pixel_tile_bounds.size(), glm::u64vec2(current_tile_image_size))));

        // Pixel bounds relative to the target image texture region.
        const glm::i64vec2 tile_target_position = glm::i64vec2(pixel_tile_bounds.min) - glm::i64vec2(target_image_region.full.min);
        const radix::geometry::Aabb2i64 target_pixel_tile_bounds(tile_target_position, tile_target_position + glm::i64vec2(pixel_tile_bounds.size()));

        // Resize current tile image and copy into image buffer.
        copy_paste_image(image, tile_image, target_pixel_tile_bounds, true /* allows und handles overflow */, rescale_filter);

    }

    cv::flip(image, image, 0);

    // Map the content region into the flipped image
    const radix::geometry::Aabb2ui content(
        glm::uvec2(target_image_region.content.min.x, image_size.y - target_image_region.content.max.y),
        glm::uvec2(target_image_region.content.max.x, image_size.y - target_image_region.content.min.y));

    return AssembledTexture{
        .image = image,
        .content = content};
}

/// Creates a texture for the given region.
[[nodiscard]]
inline alp::Result<std::optional<AssembledTexture>, TileError> assemble_texture_from_tiles(
    /// The Web Mercator bounds for which texture data should be created.
    const radix::tile::SrsBounds& target_bounds,
    /// Provider class that maps tile ids to their textures.
    const TileProvider& tile_provider,
    /// The maximal zoom level to be considered.
    const uint32_t max_zoom = gis::webmercator::max_zoom_level,
    /// The filter used to rescale the tile images if required due to missing detail tiles.
    const cv::InterpolationFlags rescale_filter = cv::INTER_LINEAR,
    /// Minimum padding to keep around the requested region.
    const glm::uvec2 min_padding = TexturePadding,
    /// Alignment of the assembled region.
    const uint32_t alignment = TextureAlignment) {
    if (target_bounds.width() == 0 || target_bounds.height() == 0) {
        LOG_WARN("Texture target bounds are empty");
        return std::nullopt;
    }

    // Find the smallest tile (id) that encompasses these bounds.
    const std::optional<radix::tile::Id> encompassing_tile_opt = find_smallest_encompassing_tile(target_bounds);
    if (!encompassing_tile_opt.has_value()) {
        LOG_WARN("Texture target bounds lie outside the tile grid");
        return std::nullopt;
    }
    radix::tile::Id smallest_encompassing_tile = encompassing_tile_opt.value();
    LOG_TRACE("Smallest encompassing tile for texture bounds is {}", radix::tile::to_string(smallest_encompassing_tile));

    if (smallest_encompassing_tile.zoom_level > max_zoom) {
        return std::nullopt;
    }

    // The smallest encompassing tile can be deeper than what is actually available, in which case walk up to the nearest available ancestor tile.
    while (smallest_encompassing_tile.zoom_level > 0 && !tile_provider.has_tile(smallest_encompassing_tile)) {
        smallest_encompassing_tile = smallest_encompassing_tile.parent();
    }

    // Very small target bounds can push the examined zoom level arbitrarily deep, but
    // gis::webmercator::compute_tile_bounds is only valid up to this zoom level; never recurse deeper than that.
    const uint32_t clamped_max_zoom = std::min(max_zoom, gis::webmercator::max_zoom_level);

    // The padding reaches past the requested bounds, so select tiles for slightly grown bounds.
    const radix::tile::SrsBounds selection_bounds = min_padding == glm::uvec2(0)
        ? target_bounds
        : radix::tile::SrsBounds(geometry::pad_bounds_relative(target_bounds, TextureSelectionGrowth));

    // Find relevant tiles in bounds
    std::vector<radix::tile::Id> tiles_to_splatter = find_relevant_tiles_to_splatter_in_bounds(
        smallest_encompassing_tile, selection_bounds, tile_provider, clamped_max_zoom);
    drop_coarse_fallback_tiles(tiles_to_splatter);
    LOG_TRACE("Found {} relevant texture tiles", tiles_to_splatter.size());

    // If we found to relevant tiles, we are done.
    if (tiles_to_splatter.empty()) {
        LOG_WARN("Found no tile textures to assemble.");
        return std::nullopt;
    }

    // Splatter tiles into texture buffer
    ALP_TRY(
        AssembledTexture texture,
        splatter_tiles_to_texture(smallest_encompassing_tile, target_bounds, tile_provider, tiles_to_splatter, rescale_filter, min_padding, alignment));
    return texture;
}
}
