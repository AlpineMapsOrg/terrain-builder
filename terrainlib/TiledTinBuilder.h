/*****************************************************************************
 * Alpine Terrain Builder
 * Copyright (C) 2022  alpinemaps.org
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

#ifndef TILEDTINBUILDER_H
#define TILEDTINBUILDER_H

#include <string>

#include "Tiler.h"
#include "ctb/Grid.hpp"

class TiledTinBuilder
{
public:
  TiledTinBuilder(const std::string& output_data_path, const std::string& input_data_path, const ctb::Grid& grid, const Tiler& tiler);
  [[nodiscard]] static TiledTinBuilder make(const std::string& output_data_path, const std::string& input_data_path, ctb::Grid::Srs srs, Tiler::Scheme tiling_scheme, Tiler::Border border);

private:
  std::string m_output_data_path;
  std::string m_input_data_path;
  ctb::Grid m_grid;
  Tiler m_tiler;
};

#endif // TILEDTINBUILDER_H
