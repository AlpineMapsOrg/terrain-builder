#pragma once

#include <memory>

#include "tntn/Mesh.h"
#include "tntn/Raster.h"

namespace tntn {

std::unique_ptr<Mesh> generate_tin_curvature(const RasterDouble& raster, double threshold);
std::unique_ptr<Mesh> generate_tin_dense_quadwalk(const RasterDouble& raster, int step = 1);
std::unique_ptr<Mesh> generate_tin_dense_quadwalk(const RasterDouble& raster, unsigned vertices_per_column, unsigned vertices_per_row);

} // namespace tntn
