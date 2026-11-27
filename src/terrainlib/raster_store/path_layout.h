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

#include "raster_store/path_layout/zoom_x_y_google.h"
#include "store/path_layout.h"

namespace raster_store::path_layout {

inline std::array<store::path_layout::Layout<radix::tile::Id>, 1> all() { return { zoom_x_y_google::layout() }; }

inline std::optional<store::path_layout::Layout<radix::tile::Id>> resolve_layout(const std::string_view id)
{
    for (const store::path_layout::Layout<radix::tile::Id> layout : all()) {
        if (layout.id == id) {
            return layout;
        }
    }
    return std::nullopt;
}

} // namespace raster_store::path_layout
