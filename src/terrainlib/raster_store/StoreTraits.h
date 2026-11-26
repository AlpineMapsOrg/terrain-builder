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
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

#include <radix/tile.h>

#include "fmt_impls.h"

namespace raster_store {

inline constexpr unsigned max_zoom_level = std::numeric_limits<uint32_t>::digits;

struct KeyTraits {
    using Key = radix::tile::Id;
    using Hasher = Key::Hasher;

    static bool is_valid(const Key& key)
    {
        if (key.zoom_level > max_zoom_level) {
            return false;
        }
        if (key.zoom_level == max_zoom_level) {
            return true;
        }
        const uint32_t extent = uint32_t { 1 } << key.zoom_level;
        return key.coords.x < extent && key.coords.y < extent;
    }
};

struct HierarchyTraits {
    using Key = radix::tile::Id;

    static Key root() { return { 0, { 0, 0 } }; }
    static std::optional<Key> parent(const Key& key)
    {
        if (key.zoom_level == 0) {
            return std::nullopt;
        }
        return key.parent();
    }
    static std::optional<std::array<Key, 4>> children(const Key& key)
    {
        if (key.zoom_level >= max_zoom_level) {
            return std::nullopt;
        }
        return key.children();
    }
};

} // namespace raster_store
