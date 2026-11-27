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
#include <string>

#include <fmt/format.h>

#include "octree/Id.h"
#include "store/path_layout.h"
#include "string_utils.h"

namespace octree::path_layout::flat {

inline std::filesystem::path locate_node(const Id& key) { return fmt::format("{}-{}", key.level(), key.index_on_level()); }

inline std::optional<Id> identify_node(const std::filesystem::path& node_path) {
    if (node_path.has_parent_path()) {
        return std::nullopt;
    }
    const std::filesystem::path::string_type& name = node_path.native();
    const size_t separator = name.find('-');
    if (separator == std::filesystem::path::string_type::npos) {
        return std::nullopt;
    }
    const std::optional<Id::Level> level = from_chars<Id::Level>(name.substr(0, separator));
    const std::optional<Id::Index> index = from_chars<Id::Index>(name.substr(separator + 1));
    if (!level.has_value() || !index.has_value()) {
        return std::nullopt;
    }
    return Id::try_make(level.value(), index.value());
}

inline store::path_layout::Layout<Id> layout() { return { "flat", locate_node, identify_node }; }

} // namespace octree::path_layout::flat
