#pragma once

#include "Mesh.h"
#include "Raster.h"
#include "SurfacePoints.h"

#include <memory>

namespace tntn {

std::unique_ptr<Mesh> generate_tin_terra(std::unique_ptr<RasterDouble> raster, double max_error);

std::unique_ptr<Mesh> generate_tin_terra(std::unique_ptr<SurfacePoints> surface_points,
    double max_error);
std::unique_ptr<Mesh> generate_tin_terra(const SurfacePoints& surface_points, double max_error);

} // namespace tntn
