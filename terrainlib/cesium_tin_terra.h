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

#ifndef CESIUM_TIN_TERRA_H
#define CESIUM_TIN_TERRA_H

#include "ParallelTileGenerator.h"
#include "tntn/geometrix.h"

namespace tntn {
class Mesh;
}

namespace cesium_tin_terra
{
class TileWriter : public ParallelTileWriterInterface {
public:
  TileWriter(Tiler::Border border) : ParallelTileWriterInterface(border, "terrain") {}
  static tntn::BBox3D computeBbox(const ctb::CRSBounds& srs_bounds, const HeightData& heights_in_metres);
  static std::unique_ptr<tntn::Mesh> toMesh(const OGRSpatialReference& srs, const ctb::CRSBounds& srs_bounds, const HeightData& heights_in_metres, bool scale_to_unit_range);
  static std::unique_ptr<tntn::Mesh> toMesh(const OGRSpatialReference& srs, const tntn::BBox3D& srs_bbox, const HeightData& heights_in_metres, bool scale_to_unit_range, unsigned simple_mesh = false);
  void write(const std::string& file_path, const Tile& tile, const HeightData& heights_in_metres) const override;
};

[[nodiscard]] ParallelTileGenerator make_generator(const std::string& output_data_path, const std::string& input_data_path, ctb::Grid::Srs srs, Tiler::Scheme tiling_scheme, Tiler::Border border);
};

#endif // CESIUM_TIN_TERRA_H
