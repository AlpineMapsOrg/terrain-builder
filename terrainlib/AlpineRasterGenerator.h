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

#ifndef ALPINERASTERGENERATOR_H
#define ALPINERASTERGENERATOR_H

#include <memory>
#include <string>

#include <glm/glm.hpp>
#include <vector>

#include "Image.h"
#include "ctb/Grid.hpp"
#include "Tile.h"
#include "Tiler.h"
#include "ctb/types.hpp"

class Dataset;
using DatasetPtr = std::shared_ptr<Dataset>;

class AlpineRasterGenerator
{
public:
  AlpineRasterGenerator(const std::string& output_data_path, const std::string& input_data_path, const ctb::Grid& grid, const Tiler& tiler);
  [[nodiscard]] static AlpineRasterGenerator make(const std::string& output_data_path, const std::string& input_data_path, ctb::Grid::Srs srs, Tiler::Scheme tiling_scheme, Tiler::Border border);

  [[nodiscard]] static glm::u8vec3 convert(float height);
  [[nodiscard]] static RgbImage convertHeights(const HeightData& heights);
  void write(const ctb::TilePoint& tilepoint, ctb::i_zoom zoom, const HeightData& heights) const;
  [[nodiscard]] std::vector<Tile> listTiles() const;
  [[nodiscard]] std::vector<Tile> listTiles(const std::pair<ctb::i_zoom, ctb::i_zoom>& zoom_range) const;
  void process() const;
  void process(const std::pair<ctb::i_zoom, ctb::i_zoom>& zoom_range) const;

protected:
  [[nodiscard]] ctb::i_zoom maxZoom() const;


private:
  std::string m_output_data_path;
  std::string m_input_data_path;
  ctb::Grid m_grid;
  Tiler m_tiler;

};

#endif // ALPINERASTERGENERATOR_H
