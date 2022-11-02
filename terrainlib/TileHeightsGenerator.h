/*****************************************************************************
 * Alpine Terrain Builder
 * Copyright (C) 2022 madam
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
#include <string>

#include "Tile.h"
#include "ctb/Grid.hpp"


class TileHeightsGenerator
{
    std::string m_input_data_path;
    ctb::Grid::Srs m_srs;
    Tile::Scheme m_scheme;
    Tile::Border m_border;
    std::filesystem::path m_output_path;
public:
    TileHeightsGenerator(std::string  input_data_path, ctb::Grid::Srs srs, Tile::Scheme scheme, Tile::Border border, std::filesystem::path output_path);
    void run(unsigned max_zoom_level) const;
};

