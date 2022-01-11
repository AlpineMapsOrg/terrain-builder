/*****************************************************************************
 * Alpine Terrain Builder
 * Copyright (C) 2022 alpinemaps.org
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

#ifndef PARALLELTILEGENERATOR_H
#define PARALLELTILEGENERATOR_H

#include <memory>

#include "Image.h"
#include "ctb/Grid.hpp"
#include "Tile.h"
#include "Tiler.h"
#include "ctb/types.hpp"

class ParallelTileWriterInterface;

class ParallelTileGenerator
{
  std::string m_output_data_path;
  std::string m_input_data_path;
  ctb::Grid m_grid;
  Tiler m_tiler;
  std::unique_ptr<ParallelTileWriterInterface> m_tile_writer;

public:
  ParallelTileGenerator(const std::string& input_data_path, const ctb::Grid& grid, const Tiler& tiler, std::unique_ptr<ParallelTileWriterInterface> tile_writer, const std::string& output_data_path);
  [[nodiscard]] static ParallelTileGenerator make(const std::string& input_data_path, ctb::Grid::Srs srs, Tiler::Scheme tiling_scheme, std::unique_ptr<ParallelTileWriterInterface> tile_writer, const std::string& output_data_path);
  void process(const std::pair<ctb::i_zoom, ctb::i_zoom>& zoom_range) const;
};

class ParallelTileWriterInterface {
  Tiler::Border m_format_requires_border;
public:
  ParallelTileWriterInterface(Tiler::Border format_requires_border) : m_format_requires_border(format_requires_border) {}
  ParallelTileWriterInterface(const ParallelTileWriterInterface&) = default;
  ParallelTileWriterInterface(ParallelTileWriterInterface&&) = default;
  virtual ~ParallelTileWriterInterface() = default;
  ParallelTileWriterInterface& operator=(const ParallelTileWriterInterface&) = default;
  ParallelTileWriterInterface& operator=(ParallelTileWriterInterface&&) = default;
  virtual void write(const std::string& base_path, const Tile& tile, const HeightData& heights) const = 0;
  [[nodiscard]] Tiler::Border formatRequiresBorder() const;
};


#endif // PARALLELTILEGENERATOR_H
