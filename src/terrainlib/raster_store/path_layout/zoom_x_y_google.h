/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
 * Copyright (C) 2026 Martin Braunsperger
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

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "range_utils.h"
#include "raster_store/StoreTraits.h"
#include "store/path_layout.h"
#include "string_utils.h"

namespace raster_store::path_layout::zoom_x_y_google {

inline std::filesystem::path locate_node(const radix::tile::Id& key)
{
    return std::to_string(key.zoom_level) + "/" + std::to_string(key.coords.x) + "/" + std::to_string(key.coords.y);
}

inline std::optional<radix::tile::Id> identify_node(const std::filesystem::path& node_path)
{
    const std::vector<std::filesystem::path> parts = to_vector(node_path);
    if (parts.size() != 3) {
        return std::nullopt;
    }
    const std::optional<std::uint32_t> zoom = from_chars<std::uint32_t>(parts[0].native());
    const std::optional<std::uint32_t> x = from_chars<std::uint32_t>(parts[1].native());
    const std::optional<std::uint32_t> y = from_chars<std::uint32_t>(parts[2].native());
    if (!zoom.has_value() || !x.has_value() || !y.has_value()) {
        return std::nullopt;
    }
    const radix::tile::Id key { zoom.value(), { x.value(), y.value() } };
    return KeyTraits::is_valid(key) ? std::optional(key) : std::nullopt;
}

inline store::path_layout::Layout<radix::tile::Id> layout() { return { "zoom/x/y_google", locate_node, identify_node }; }

} // namespace raster_store::path_layout::zoom_x_y_google
