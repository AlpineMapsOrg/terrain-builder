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

#include <fstream>
#include <iterator>

#include "TileHeights.h"

namespace {
auto key(const Tile::Id& tile_id)
{
    const auto id = tile_id.to(Tile::Scheme::Tms);
    return std::make_tuple(id.zoom_level, id.coords.x, id.coords.y);
}
}

TileHeights::TileHeights()
{

}

void TileHeights::emplace(const Tile::Id& tile_id, const std::pair<float, float>& min_max)
{
    m_data[key(tile_id)] = min_max;
}

TileHeights::ValueType TileHeights::query(Tile::Id tile_id) const
{
    auto iter = m_data.find(key(tile_id));
    while (iter == m_data.end()) {
        tile_id = tile_id.parent();
        iter = m_data.find(key(tile_id));
        if (tile_id.zoom_level == unsigned(-1)) {
            assert(false);
            return { 0, 9000 }; // mount everest is a bit under 9km, but there should always be a root tile.
        }
    }
    return iter->second;
}

void TileHeights::write_to(const std::filesystem::__cxx11::path& path) const
{
    std::vector<std::pair<KeyType, ValueType>> vector_data;
    vector_data.reserve(m_data.size());
    std::copy(m_data.cbegin(), m_data.cend(), std::back_inserter(vector_data));

    std::filesystem::create_directories(path.parent_path());
    std::ofstream file(path, std::ios::binary);

    u_int64_t size = vector_data.size();
    file << size;
    file.write(reinterpret_cast<char*>(vector_data.data()), sizeof(decltype(vector_data.front())) * vector_data.size());
}

TileHeights TileHeights::read_from(const std::filesystem::__cxx11::path& path)
{
    using DataType = std::pair<KeyType, ValueType>;
    std::ifstream file(path, std::ios::binary);
    u_int64_t size = 0;
    file >> size;
    std::vector<DataType> vector_data;
    vector_data.resize(size);
    file.read(reinterpret_cast<char*>(vector_data.data()), sizeof(decltype(vector_data.front())) * size);
    TileHeights new_heights;
    for (const auto& entry : vector_data) {
        new_heights.m_data[entry.first] = entry.second;
    }
    return new_heights;
}
