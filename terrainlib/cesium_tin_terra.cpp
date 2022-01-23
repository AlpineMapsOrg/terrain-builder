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

#include "cesium_tin_terra.h"

#include <memory>

#include "Image.h"
#include "tntn/Raster.h"
#include "tntn/geometrix.h"
#include "tntn/terra_meshing.h"
#include "tntn/QuantizedMeshIO.h"


std::unique_ptr<tntn::Mesh> cesium_tin_terra::TileWriter::toMesh(const ctb::CRSBounds& srs_bounds, const HeightData& heights)
{
  const auto max_error = srs_bounds.getWidth() / heights.width();
  auto raster = std::make_unique<tntn::RasterDouble>(image::transformImage(heights, [](auto v) { return double(v); }), srs_bounds);
  auto mesh = tntn::generate_tin_terra(std::move(raster), max_error);

  mesh->generate_triangles();
  return mesh;
}

void cesium_tin_terra::TileWriter::write(const std::string& file_path, const Tile& tile, const HeightData& heights) const
{
  const auto mesh = cesium_tin_terra::TileWriter::toMesh(tile.srsBounds, heights);
  tntn::BBox3D bbox;
  mesh->get_bbox(bbox);
  tntn::write_mesh_as_qm(file_path.c_str(), *mesh, bbox, false, true);
}

ParallelTileGenerator cesium_tin_terra::make_generator(const std::string& output_data_path, const std::string& input_data_path, ctb::Grid::Srs srs, Tiler::Scheme tiling_scheme, Tiler::Border border)
{
  return ParallelTileGenerator::make(input_data_path, srs, tiling_scheme, std::make_unique<cesium_tin_terra::TileWriter>(border), output_data_path);
}
