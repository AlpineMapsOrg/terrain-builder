/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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
#include <cstdint>
#include <optional>
#include <vector>

#include "raster/ClampedView.h"
#include "raster/algorithm/copy.h"
#include "raster_store/scaler.h"
#include "raster_store/storage.h"

/// A half-open vertex tile owns its northwestern corner. Its eastern column,
/// southern row and southeastern corner, the endpoints of linear interpolation
/// in its last interval, belong to its neighbours. These functions read such
/// endpoints from disjoint vertex snapshots with bounded, non-recursive lookups.
namespace raster_store::vertex_grid {

using Key = radix::tile::Id;

// How a coarser tile covering a region completes its own grid for linear interpolation.
enum class Completion {
    // Look up its endpoints in its neighbours.
    Lookup,
    // Repeat its border, except at a known vertex.
    Repeat,
};

// A vertex with a known value, in the pixels of a region's tile.
template <typename PixelType>
struct KnownVertex {
    glm::ivec2 position;
    PixelType value;
    std::uint16_t attribution;
};

namespace detail {
    // The same-zoom tile at the step, wrapping at the antimeridian; none beyond the poles.
    inline std::optional<Key> adjacent(const Key& key, glm::ivec2 step)
    {
        const auto tiles = std::int64_t(1) << key.zoom_level;
        const auto y = std::int64_t(key.coords.y) + step.y;
        if (y < 0 || y >= tiles)
            return std::nullopt;
        return Key { key.zoom_level, { unsigned((std::int64_t(key.coords.x) + step.x + tiles) % tiles), unsigned(y) } };
    }

    // The tile with its border repeated as endpoint column, row and corner.
    template <typename PixelType>
    Expected<Tile<PixelType>> repeat_border(const Tile<PixelType>& tile)
    {
        const auto size = tile.data.size() + glm::uvec2(1);
        Tile<PixelType> result(0);
        auto data = raster::algorithm::copy(Error::asserting_unwrap(raster::make_clamped_view(tile.data, glm::ivec2(0), size)));
        auto attribution = raster::algorithm::copy(Error::asserting_unwrap(raster::make_clamped_view(tile.attribution, glm::ivec2(0), size)));
        if (!data)
            return Error::propagate(std::move(data));
        if (!attribution)
            return Error::propagate(std::move(attribution));
        result.data = std::move(*data);
        result.attribution = std::move(*attribution);
        return result;
    }

    // The status of an indexed key, with Inner treated as an error of a disjoint snapshot.
    inline Expected<std::optional<store::NodeStatus>> status(const store::Index<StoreTraits>& index, const Key& key)
    {
        auto result = index.get(key);
        if (result && *result == store::NodeStatus::Inner)
            return Error::fail(Error::Code::InvalidInput, "vertex endpoint lookup requires a disjoint snapshot, not an inner tile at " + to_string(key));
        return result;
    }
} // namespace detail

template <typename PixelType>
Expected<Tile<PixelType>> closed(
    const storage::IndexedStorage<PixelType>& storage, unsigned side, pixel::Mapping mapping, const Key& key, const Tile<PixelType>& tile);

/// Supplies [origin, origin + size) of the same-zoom tile `region`, which need not be
/// physical, to the views: a physical tile at that zoom is copied, immediate physical
/// children of a virtual one are decimated, and a physical coarser tile covering it is
/// interpolated linearly, completing its grid as requested. Returns whether any source
/// supplied pixels; unsupplied pixels are left unchanged.
template <typename PixelType>
Expected<bool> supply(const storage::IndexedStorage<PixelType>& storage,
    unsigned side,
    pixel::Mapping mapping,
    const Key& region,
    glm::uvec2 origin,
    glm::uvec2 size,
    raster::View<PixelType> data,
    raster::View<std::uint16_t> attribution,
    Completion completion,
    const std::optional<KnownVertex<PixelType>>& known = std::nullopt)
{
    constexpr auto cheapest = raster::algorithm::Resampling::Cheapest;
    constexpr auto vertex = pixel::Placement::Vertex;
    const auto& index = storage.index();
    auto status = detail::status(index, region);
    if (!status)
        return Error::propagate(std::move(status));
    if (*status == store::NodeStatus::Virtual) {
        const unsigned extent = side / 2;
        bool supplied = false;
        for (const auto& child : region.children()) {
            const auto child_origin = (child.coords & glm::uvec2(1)) * extent;
            const auto first = glm::max(origin, child_origin);
            const auto last = glm::min(origin + size, child_origin + extent);
            if (glm::any(glm::greaterThanEqual(first, last)))
                continue;
            auto child_status = detail::status(index, child);
            if (!child_status)
                return Error::propagate(std::move(child_status));
            if (*child_status != store::NodeStatus::Leaf)
                continue;
            auto source = storage.load(child);
            if (!source)
                return Error::propagate(std::move(source), "read vertex endpoint child " + to_string(child));
            const auto output = Error::asserting_unwrap(raster::make_view(data, first - origin, last - first));
            const auto output_attribution = Error::asserting_unwrap(raster::make_view(attribution, first - origin, last - first));
            if (auto scaled = raster_store::scale(
                    source->data, source->attribution, 0, -1, cheapest, glm::i64vec2(first - child_origin), mapping, output, output_attribution, vertex);
                !scaled)
                return Error::propagate(std::move(scaled));
            supplied = true;
        }
        return supplied;
    }
    std::optional<Key> covering;
    for (std::optional<Key> current = region; current && !covering; current = StoreTraits::parent(*current)) {
        auto current_status = detail::status(index, *current);
        if (!current_status)
            return Error::propagate(std::move(current_status));
        if (*current_status == store::NodeStatus::Leaf)
            covering = current;
    }
    if (!covering)
        return false;
    auto source = storage.load(*covering);
    if (!source)
        return Error::propagate(std::move(source), "read vertex endpoint source " + to_string(*covering));
    const unsigned gap = region.zoom_level - covering->zoom_level;
    // The region's position in the covering tile upscaled to the region's spacing.
    const auto base = (glm::i64vec2(region.coords) - (glm::i64vec2(covering->coords) << std::int64_t(gap))) * std::int64_t(side);
    const auto offset = base + glm::i64vec2(origin);
    if (raster::algorithm::required_source_window(source->data.size(), 0, int(gap), cheapest, offset, size, vertex)) {
        if (auto scaled = raster_store::scale(source->data, source->attribution, 0, int(gap), cheapest, offset, mapping, data, attribution, vertex); !scaled)
            return Error::propagate(std::move(scaled));
        return true;
    }
    // The interpolation reaches the covering tile's endpoints.
    auto grid = completion == Completion::Lookup ? closed(storage, side, mapping, *covering, *source) : detail::repeat_border(*source);
    if (!grid)
        return Error::propagate(std::move(grid));
    if (completion == Completion::Repeat && known) {
        const auto position = base + glm::i64vec2(known->position);
        const std::int64_t factor = std::int64_t(1) << gap;
        const auto vertex_position = position / factor;
        if (position % factor == glm::i64vec2(0) && glm::all(glm::lessThanEqual(vertex_position, glm::i64vec2(side)))
            && glm::any(glm::equal(vertex_position, glm::i64vec2(side)))) {
            grid->data.pixel(glm::uvec2(vertex_position)) = known->value;
            grid->attribution.pixel(glm::uvec2(vertex_position)) = known->attribution;
        }
    }
    if (auto scaled = raster_store::scale(grid->data, grid->attribution, 0, int(gap), cheapest, offset, mapping, data, attribution, vertex); !scaled)
        return Error::propagate(std::move(scaled));
    return true;
}

/// The physical vertex tile `key` with its endpoint column, row and corner. The
/// southeastern neighbour supplies the corner first; the eastern and southern ones
/// supply the column and row, where a coarser tile repeats its own border except at
/// that corner. A column or row without a source repeats the tile's border, also
/// beyond the southern world edge; a corner without a source repeats the completed
/// column's last value.
template <typename PixelType>
Expected<Tile<PixelType>> closed(
    const storage::IndexedStorage<PixelType>& storage, unsigned side, pixel::Mapping mapping, const Key& key, const Tile<PixelType>& tile)
{
    auto result = detail::repeat_border(tile);
    if (!result)
        return result;
    const auto strip = [&](glm::ivec2 step, glm::uvec2 size, const std::optional<KnownVertex<PixelType>>& known) -> Expected<bool> {
        const auto neighbour = detail::adjacent(key, step);
        if (!neighbour)
            return false;
        const auto destination = glm::uvec2(step) * side;
        return supply(storage,
            side,
            mapping,
            *neighbour,
            glm::uvec2(0),
            size,
            Error::asserting_unwrap(raster::make_view(result->data, destination, size)),
            Error::asserting_unwrap(raster::make_view(result->attribution, destination, size)),
            Completion::Repeat,
            known);
    };
    auto corner_supplied = strip({ 1, 1 }, glm::uvec2(1), std::nullopt);
    if (!corner_supplied)
        return Error::propagate(std::move(corner_supplied));
    std::optional<KnownVertex<PixelType>> corner;
    if (*corner_supplied)
        corner = KnownVertex<PixelType> { {}, result->data.pixel(glm::uvec2(side)), result->attribution.pixel(glm::uvec2(side)) };
    const auto at = [&](glm::ivec2 position) {
        auto known = corner;
        if (known)
            known->position = position;
        return known;
    };
    // In the neighbour's pixels, the corner lies below the eastern column and right of the southern row.
    if (auto supplied = strip({ 1, 0 }, { 1, side }, at({ 0, int(side) })); !supplied)
        return Error::propagate(std::move(supplied));
    if (auto supplied = strip({ 0, 1 }, { side, 1 }, at({ int(side), 0 })); !supplied)
        return Error::propagate(std::move(supplied));
    if (!corner) {
        result->data.pixel(glm::uvec2(side)) = result->data.pixel({ side, side - 1 });
        result->attribution.pixel(glm::uvec2(side)) = result->attribution.pixel({ side, side - 1 });
    }
    return result;
}

/// The physical tiles `closed` may read for the endpoints of `key`, sorted and unique,
/// without reading payloads.
inline Expected<std::vector<Key>> endpoint_sources(const store::Index<StoreTraits>& index, const Key& key)
{
    std::vector<Key> result;
    for (const auto step : { glm::ivec2(1, 0), glm::ivec2(0, 1), glm::ivec2(1, 1) }) {
        const auto neighbour = detail::adjacent(key, step);
        if (!neighbour)
            continue;
        auto status = detail::status(index, *neighbour);
        if (!status)
            return Error::propagate(std::move(status));
        if (*status == store::NodeStatus::Virtual) {
            // The children along the neighbour's western column or northern row.
            for (const auto& child : neighbour->children()) {
                if ((step.x == 1 && (child.coords.x & 1) != 0) || (step.y == 1 && (child.coords.y & 1) != 0))
                    continue;
                auto child_status = detail::status(index, child);
                if (!child_status)
                    return Error::propagate(std::move(child_status));
                if (*child_status == store::NodeStatus::Leaf)
                    result.push_back(child);
            }
            continue;
        }
        for (std::optional<Key> current = neighbour; current; current = StoreTraits::parent(*current)) {
            auto current_status = detail::status(index, *current);
            if (!current_status)
                return Error::propagate(std::move(current_status));
            if (*current_status == store::NodeStatus::Leaf) {
                result.push_back(*current);
                break;
            }
        }
    }
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

} // namespace raster_store::vertex_grid
