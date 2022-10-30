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

#include <algorithm>
#include <memory>

#include "Image.h"
#include "ctb/types.hpp"
#include "srs.h"
#include "tntn/Mesh.h"
#include "tntn/Raster.h"
#include "tntn/geometrix.h"
#include "tntn/terra_meshing.h"
#include "tntn/simple_meshing.h"
#include "tntn/QuantizedMeshIO.h"

tntn::BBox3D cesium_tin_terra::TileWriter::computeBbox(const ctb::CRSBounds& srs_bounds, const HeightData& heights_in_metres) {
  const auto [min_height, max_height] = std::ranges::minmax_element(heights_in_metres);
  tntn::BBox3D bbox;
  bbox.min.x = srs_bounds.getMinX();
  bbox.min.y = srs_bounds.getMinY();
  bbox.min.z = double(*min_height);
  bbox.max.x = srs_bounds.getMaxX();
  bbox.max.y = srs_bounds.getMaxY();
  bbox.max.z = double(*max_height);
  if (std::abs(bbox.min.z - bbox.max.z) < 0.1)
    bbox.max.z = bbox.min.z + 0.1;
  return bbox;
}

std::unique_ptr<tntn::Mesh> cesium_tin_terra::TileWriter::toMesh(const OGRSpatialReference& srs, const ctb::CRSBounds& srs_bounds, const HeightData& heights_in_metres, bool scale_to_unit_range, unsigned simple_mesh)
{
  return toMesh(srs, computeBbox(srs_bounds, heights_in_metres), heights_in_metres, scale_to_unit_range, simple_mesh);
}

std::unique_ptr<tntn::Mesh> cesium_tin_terra::TileWriter::toMesh(const OGRSpatialReference& srs, const tntn::BBox3D& bbox, const HeightData& heights_in_metres, bool scale_to_unit_range, unsigned simple_mesh)
{
  auto bbox_copy = bbox;
  bbox_copy.min.z = bbox.max.z;

  const auto ecef_bbox = srs::toECEF(srs, bbox_copy);   // we are not considering the height difference, so using bbox.max.z for min as well
  const auto tile_size = heights_in_metres.width();
  const auto grid_size = tile_size - 1;

  const auto diagonal_distance_in_m = distance(ecef_bbox.min, ecef_bbox.max);
  auto max_error = 1.0 * (diagonal_distance_in_m / std::sqrt(tile_size * tile_size * 2.0));
  assert(bbox.max.z - bbox.min.z >= 0);

  std::unique_ptr<tntn::RasterDouble> raster;
  if (scale_to_unit_range) {
    const auto scale = std::max(bbox.max.z - bbox.min.z, 1.0);
    const auto offset = bbox.min.z;
    max_error = max_error / scale;
    const auto corrected_bounds = ctb::CRSBounds{0.0, 0.0, 1.0 + 1.0 / grid_size, 1.0 + 1.0 / grid_size};
    raster = std::make_unique<tntn::RasterDouble>(image::transformImage(heights_in_metres, [=](auto v) { return (double(v) - offset) / scale; }), corrected_bounds);
  }
  else {
    const auto correction = tntn::xy((bbox.max - bbox.min) * (1.0 / grid_size));
    const auto corrected_bounds = ctb::CRSBounds{bbox.min.x, bbox.min.y, bbox.max.x + correction.x, bbox.max.y + correction.y};
    raster = std::make_unique<tntn::RasterDouble>(image::transformImage(heights_in_metres, [=](auto v) { return double(v);}), corrected_bounds);
  }
  assert(max_error > 0);
  const auto dbg_heights = raster->asVector();
  std::unique_ptr<tntn::Mesh> mesh;
  if (simple_mesh == 0)
    mesh = tntn::generate_tin_terra(std::move(raster), max_error);
  else
    mesh = tntn::generate_tin_dense_quadwalk(*raster, simple_mesh, simple_mesh);

  const auto poly_count = mesh->poly_count();
  const auto vertex_count = mesh->vertices().size();

  mesh->generate_triangles();
  return mesh;
}

void cesium_tin_terra::TileWriter::write(const std::string& file_path, const Tile& tile, const HeightData& heights) const
{
  const auto scale_to_unit_range = false;
  const auto simple_mesh_resolution = [](ctb::i_zoom zoom) -> unsigned {
    switch (zoom) {
    case 0: return 33;
    case 1: return 33;
    case 2: return 17;
    case 3: return 9;
    case 4: return 5;
    case 5: return 3;
    default: return 0;
    }
  };
  const auto srs_bbox = computeBbox(tile.srsBounds, heights);

  // need to create this per thread, as the object doesn't seem to be reentrant.
  OGRSpatialReference srs;
  srs.importFromEPSG(tile.srs_epsg);
  srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  const auto mesh = cesium_tin_terra::TileWriter::toMesh(srs, srs_bbox, heights, scale_to_unit_range, simple_mesh_resolution(tile.zoom));
  const auto& vertices = mesh->vertices_as_vector();
  tntn::write_mesh_as_qm(file_path.c_str(), *mesh, srs_bbox, srs, scale_to_unit_range, true);
}

ParallelTileGenerator cesium_tin_terra::make_generator(const std::string& input_data_path, const std::string& output_data_path, ctb::Grid::Srs srs, ParallelTiler::Scheme tiling_scheme, ParallelTiler::Border border)
{
  return ParallelTileGenerator::make(input_data_path, srs, tiling_scheme, std::make_unique<cesium_tin_terra::TileWriter>(border), output_data_path);
}
