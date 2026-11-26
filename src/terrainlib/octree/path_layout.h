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

#include <array>
#include <optional>
#include <string_view>

#include "octree/Id.h"
#include "octree/path_layout/flat.h"
#include "octree/path_layout/level_and_coordinate_directories.h"
#include "store/path_layout.h"

namespace octree::path_layout {

inline std::array<store::path_layout::Layout<Id>, 2> all() { return { flat::layout(), level_and_coordinate_directories::layout() }; }

inline std::optional<store::path_layout::Layout<Id>> resolve_layout(const std::string_view id) {
    for (const store::path_layout::Layout<Id> layout : all()) {
        if (layout.id == id) {
            return layout;
        }
    }
    return std::nullopt;
}

} // namespace octree::path_layout
