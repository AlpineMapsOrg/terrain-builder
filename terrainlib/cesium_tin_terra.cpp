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
#include "tntn/MeshWriter.h"
#include "tntn/Raster.h"
#include "tntn/geometrix.h"
#include "tntn/terra_meshing.h"

namespace {
class ObjTileWriter : public ParallelTileWriterInterface {
public:
  ObjTileWriter(Tiler::Border border) : ParallelTileWriterInterface(border, "obj") {}
  void write(const std::string& file_path, const Tile& tile, const HeightData& heights) const override {
    const auto max_error = tile.srsBounds.getWidth() / tile.tileSize;
    auto raster = std::make_unique<tntn::RasterDouble>(image::transformImage(heights, [](auto v) { return double(v); }), tile);
    auto mesh = tntn::generate_tin_terra(std::move(raster), max_error);

    mesh->generate_triangles();

    tntn::BBox3D bbox;
    mesh->get_bbox(bbox);
    auto mesh_writer = tntn::ObjMeshWriter();
    mesh_writer.write_mesh_to_file(file_path.c_str(), *mesh, bbox);
  }
};
}

void cesium_tin_terra::TileWriter::write(const std::string& file_path, const Tile& tile, const HeightData& heights) const
{
  const auto max_error = tile.srsBounds.getWidth() / tile.tileSize;
  auto raster = std::make_unique<tntn::RasterDouble>(image::transformImage(heights, [](auto v) { return double(v); }), tile);
  auto mesh = tntn::generate_tin_terra(std::move(raster), max_error);

  mesh->generate_triangles();

  tntn::BBox3D bbox;
  mesh->get_bbox(bbox);
  auto mesh_writer = tntn::QuantizedMeshWriter(true);
  mesh_writer.write_mesh_to_file(file_path.c_str(), *mesh, bbox);
}

ParallelTileGenerator cesium_tin_terra::make_generator(const std::string& output_data_path, const std::string& input_data_path, ctb::Grid::Srs srs, Tiler::Scheme tiling_scheme, Tiler::Border border)
{
  return ParallelTileGenerator::make(input_data_path, srs, tiling_scheme, std::make_unique<cesium_tin_terra::TileWriter>(border), output_data_path);
}

ParallelTileGenerator cesium_tin_terra::make_objGenerator(const std::string& output_data_path, const std::string& input_data_path, ctb::Grid::Srs srs, Tiler::Scheme tiling_scheme, Tiler::Border border)
{
  return ParallelTileGenerator::make(input_data_path, srs, tiling_scheme, std::make_unique<ObjTileWriter>(border), output_data_path);
}
