/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2025 Martin Braunsperger
 * Copyright (C) 2025 Adam Celarek-Litofcenko
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

#include <cstddef>
#include <system_error>
#include <type_traits>

#include <zpp_bits.h>

#include "octree/Id.h"

namespace octree {
namespace {
constexpr zpp::bits::errc success() {
    return zpp::bits::errc(std::errc());
}
} // namespace

template <
    size_t Dimensions,
    size_t MaxLevel,
    typename LevelT,
    typename IndexT,
    typename CoordT>
constexpr auto serialize(auto &archive, octree::Id_<Dimensions, MaxLevel, LevelT, IndexT, CoordT> &id) {
    using Id = octree::Id_<Dimensions, MaxLevel, LevelT, IndexT, CoordT>;
    typename Id::Level level;
    typename Id::Index index;
    auto result = archive(level, index);
    using Result = std::remove_cvref_t<decltype(result)>;
    if constexpr (!std::is_same_v<Result, zpp::bits::errc> && !std::is_same_v<Result, std::errc>) {
        return result;
    } else {
        if (zpp::bits::failure(result)) {
            return result;
        }

        auto maybe_id = Id::try_make(level, index);
        if (!maybe_id) {
            return zpp::bits::errc(std::errc::bad_message);
        }

        id = *maybe_id;
        return success();
    }
}

template <
    size_t Dimensions,
    size_t MaxLevel,
    typename LevelT,
    typename IndexT,
    typename CoordT>
constexpr auto serialize(auto &archive, const octree::Id_<Dimensions, MaxLevel, LevelT, IndexT, CoordT> &id) {
    return archive(id.level(), id.index_on_level());
}
} // namespace octree
