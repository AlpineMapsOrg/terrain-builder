/*****************************************************************************
 * Alpine Terrain Builder
 * Copyright (C) 2022 Adam Celarek
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
#include <unordered_map>

#include "algorithms/hasher.h"
#include "Tile.h"

class TileHeights {
    using KeyType = std::tuple<unsigned, unsigned, unsigned>;
    using ValueType = std::pair<float, float>;
    std::unordered_map<KeyType, ValueType, hasher::for_tuple<unsigned, unsigned, unsigned>> m_data;
public:
    TileHeights();
    void emplace(const tile::Id& tile_id, const std::pair<float, float>& min_max);
    [[nodiscard]] ValueType query(tile::Id tile_id) const;
    void write_to(const std::filesystem::path& path) const;
    static TileHeights read_from(const std::filesystem::path& path);
};

