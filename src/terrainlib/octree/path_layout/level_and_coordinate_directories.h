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

#include <filesystem>
#include <optional>
#include <vector>

#include <fmt/format.h>

#include "octree/Id.h"
#include "store/path_layout.h"
#include "string_utils.h"

namespace octree::path_layout::level_and_coordinate_directories {

inline std::filesystem::path locate_node(const Id& key) {
    const Id::Coords coordinates = key.coords();
    return fmt::format("{}/{}/{}/{}", key.level(), coordinates.x, coordinates.y, coordinates.z);
}

inline std::optional<Id> identify_node(const std::filesystem::path& node_path) {
    const std::vector<std::filesystem::path> parts(node_path.begin(), node_path.end());
    if (parts.size() != 4) {
        return std::nullopt;
    }
    const std::optional<Id::Level> level = from_chars<Id::Level>(parts[0].native());
    const std::optional<Id::Coord> x = from_chars<Id::Coord>(parts[1].native());
    const std::optional<Id::Coord> y = from_chars<Id::Coord>(parts[2].native());
    const std::optional<Id::Coord> z = from_chars<Id::Coord>(parts[3].native());
    if (!level.has_value() || !x.has_value() || !y.has_value() || !z.has_value()) {
        return std::nullopt;
    }
    return Id::try_make(level.value(), { x.value(), y.value(), z.value() });
}

inline store::path_layout::Layout<Id> layout() { return { "level_and_coordinate_directories", locate_node, identify_node }; }

} // namespace octree::path_layout::level_and_coordinate_directories
