/*****************************************************************************
 * AlpineMaps.org
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
#include <optional>
#include <span>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include "error/Result.h"
#include "error/utils.h"
#include "numeric/checked_cast.h"
#include "octree/Id.h"

namespace cli_support {

namespace detail {

// The node that a level and an index name, if they name one.
[[nodiscard]] inline std::optional<octree::Id> try_make_from_index(const std::span<const uint64_t, 2> values) {
    const std::optional<octree::Id::Level> level_opt = try_cast<octree::Id::Level>(values[0]);
    const std::optional<octree::Id::Index> index_opt = try_cast<octree::Id::Index>(values[1]);
    if (level_opt.has_value() && index_opt.has_value()) {
        return octree::Id::try_make(*level_opt, *index_opt);
    } else {
        return std::nullopt;
    }
}

// The node that a level and coordinates name, if they name one.
[[nodiscard]] inline std::optional<octree::Id> try_make_from_coords(const std::span<const uint64_t, 4> values) {
    const std::optional<octree::Id::Level> level_opt = try_cast<octree::Id::Level>(values[0]);
    const std::optional<octree::Id::Coord> x_opt = try_cast<octree::Id::Coord>(values[1]);
    const std::optional<octree::Id::Coord> y_opt = try_cast<octree::Id::Coord>(values[2]);
    const std::optional<octree::Id::Coord> z_opt = try_cast<octree::Id::Coord>(values[3]);
    if (level_opt.has_value() && x_opt.has_value() && y_opt.has_value() && z_opt.has_value()) {
        return octree::Id::try_make(*level_opt, octree::Id::Coords(*x_opt, *y_opt, *z_opt));
    } else {
        return std::nullopt;
    }
}

} // namespace detail

// Parses the octree node that `values` name as either <level> <index> or <level> <x> <y> <z>.
[[nodiscard]] inline alp::Result<octree::Id> parse_octree_id(const std::span<const uint64_t> values) {
    std::optional<octree::Id> id_opt;
    if (values.size() == 2) {
        id_opt = detail::try_make_from_index(values.first<2>());
    } else if (values.size() == 4) {
        id_opt = detail::try_make_from_coords(values.first<4>());
    } else {
        return alp::make_error(fmt::format("expected <level> <index> or <level> <x> <y> <z>, got {} values", values.size()));
    }

    if (id_opt.has_value()) {
        return *id_opt;
    } else {
        return alp::make_error(fmt::format("'{}' does not name an octree node", fmt::join(values, " ")));
    }
}

} // namespace cli_support
