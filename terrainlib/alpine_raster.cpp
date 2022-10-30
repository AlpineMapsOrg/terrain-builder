/*****************************************************************************
 * Alpine Terrain Builder
 * Copyright (C) 2022 alpinemaps.org
 * Copyright (C) 2022 Adam Celarek <family name at cg tuwien ac at>
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

#include "alpine_raster.h"

#include <algorithm>
#include <execution>
#include <filesystem>

#include <fmt/core.h>
#include <memory>

#include "Dataset.h"
#include "DatasetReader.h"
#include "Exception.h"
#include "Image.h"
#include "ParallelTileGenerator.h"
#include "ctb/GlobalGeodetic.hpp"
#include "ctb/GlobalMercator.hpp"
#include "ctb/Grid.hpp"
#include "ctb/types.hpp"



ParallelTileGenerator alpine_raster::make_generator(const std::string& input_data_path, const std::string& output_data_path, ctb::Grid::Srs srs, ParallelTiler::Scheme tiling_scheme, ParallelTiler::Border border, unsigned grid_resolution)
{
  return ParallelTileGenerator::make(input_data_path, srs, tiling_scheme, std::make_unique<alpine_raster::TileWriter>(border), output_data_path, grid_resolution);
}

glm::u8vec3 alpine_raster::convert(float height)
{
  const auto r = std::clamp(int(height / 32.0f), 0, 255);
  const auto g = std::clamp(int(std::fmod(height, 32.0f) * 8), 0, 255);

  return {glm::u8(r), glm::u8(g), 0};
}

void alpine_raster::TileWriter::write(const std::string& file_path, const Tile& tile, const HeightData& heights) const
{
  image::saveImageAsPng(image::transformImage(heights, alpine_raster::convert), file_path);
}
