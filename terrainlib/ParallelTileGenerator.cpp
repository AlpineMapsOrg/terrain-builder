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

#include "ParallelTileGenerator.h"

#include <execution>

#include "Dataset.h"
#include "ctb/GlobalGeodetic.hpp"
#include "ctb/GlobalMercator.hpp"

#include <DatasetReader.h>

ParallelTileGenerator::ParallelTileGenerator(const std::string& input_data_path, const ctb::Grid& grid, const Tiler& tiler, std::unique_ptr<ParallelTileWriterInterface> tile_writer, const std::string& output_data_path) :
    m_output_data_path(output_data_path), m_input_data_path(input_data_path), m_grid(grid), m_tiler(tiler), m_tile_writer(std::move(tile_writer))
{

}

ParallelTileGenerator ParallelTileGenerator::make(const std::string& input_data_path, ctb::Grid::Srs srs, Tiler::Scheme tiling_scheme, std::unique_ptr<ParallelTileWriterInterface> tile_writer, const std::string& output_data_path)
{
  const auto dataset = Dataset::make_shared(input_data_path);
  ctb::Grid grid = ctb::GlobalGeodetic(256);
  if (srs == ctb::Grid::Srs::SphericalMercator)
    grid = ctb::GlobalMercator(256);
  const auto border = tile_writer->formatRequiresBorder();
  return {input_data_path, grid, Tiler(grid, dataset->bounds(grid.getSRS()), border, tiling_scheme), std::move(tile_writer), output_data_path};
}

void ParallelTileGenerator::process(const std::pair<ctb::i_zoom, ctb::i_zoom>& zoom_range) const
{
  const auto tiles = m_tiler.generateTiles(zoom_range);
  const auto fun = [this](const Tile& tile) {
    // Recreating Dataset for every tile. This was the easiest fix for multithreading,
    // and it takes only 0.5% of the time (half a percent).
    // most of the cpu time is used in 'readWithOverviews' (specificly 'RasterIO', and
    // 'VRTWarpedRasterBand::IReadBlock') and a bit in 'write' (specifically 'FreeImage_Save').
    const auto dataset = Dataset::make_shared(m_input_data_path);
    DatasetReader reader(dataset, m_grid.getSRS(), 1);
    const auto heights = reader.readWithOverviews(tile.srsBounds, tile.tileSize, tile.tileSize);
    m_tile_writer->write(m_output_data_path, tile, heights);
  };
  std::for_each(std::execution::par, tiles.begin(), tiles.end(), fun);
}

Tiler::Border ParallelTileWriterInterface::formatRequiresBorder() const
{
  return m_format_requires_border;
}
