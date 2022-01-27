#pragma once

#include "tntn/Raster.h"
#include <vector>
#include <string>
#include "tntn/geometrix.h"

namespace tntn {

namespace raster_tools
{
RasterDouble integer_downsample_mean(const RasterDouble& src, int window_size);

void flip_data_x(RasterDouble& r);

void flip_data_y(RasterDouble& r);

void find_minmax(const RasterDouble& raster, double& min, double& max);

BBox3D get_bounding_box3d(const RasterDouble& raster);

double sample_nearest_valid_avg(const RasterDouble& src,
                                const unsigned int row,
                                const unsigned int column,
                                int min_averaging_samples = 1);
};

} // namespace tntn
