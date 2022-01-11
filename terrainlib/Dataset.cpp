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

#include "Dataset.h"

#include <algorithm>
#include <cassert>
#include <memory>
#include <regex>
#include <stdexcept>

#include <gdal_priv.h>

#include "Exception.h"
#include "util.h"
#include "ctb/Grid.hpp"
#include "tntn/logging.h"
#include "tntn/gdal_init.h"

Dataset::Dataset(const std::string& path)
{
  tntn::initialize_gdal_once();
  m_gdal_dataset.reset(static_cast<GDALDataset *>(GDALOpen(path.c_str(), GA_ReadOnly)));
  if (!m_gdal_dataset) {
    TNTN_LOG_FATAL("Couldn't open dataset {}.\n", path);
    throw Exception("");
  }
  m_name = std::regex_replace(path, std::regex("^.*/"), "");
  m_name = std::regex_replace(m_name, std::regex(R"(\.\w+$)"), "");
}

Dataset::Dataset(GDALDataset* dataset)
{
  tntn::initialize_gdal_once();
  m_gdal_dataset.reset(dataset);
  if (!m_gdal_dataset) {
    TNTN_LOG_FATAL("Dataset is null.\n");
    throw Exception("Dataset is null.");
  }
}

DatasetPtr Dataset::make_shared(const std::string& path)
{
  return std::make_shared<Dataset>(path);
}

std::string Dataset::name() const
{
  return m_name;
}

Dataset::~Dataset() = default;

ctb::CRSBounds Dataset::bounds() const
{
  std::array<double, 6> adfGeoTransform = {};
  if (m_gdal_dataset->GetGeoTransform(adfGeoTransform.data()) != CE_None)
    throw Exception("Could not get transformation information from source dataset");

  // https://gdal.org/user/raster_data_model.html
  // gdal has a row/column raster format, where row 0 is the top most row.
  // an affine transform is used to convert row/column into the datasets SRS.
  // computing bounds is going first from row/column to dataset SRS and then to target SRS

  // we don't support sheering or rotation for now
  if (adfGeoTransform[2] != 0.0 || adfGeoTransform[4] != 0.0)
    throw Exception("Dataset geo transform contains sheering or rotation. This is not supported!");

  const double westX =  adfGeoTransform[0];
  const double southY = adfGeoTransform[3] + (heightInPixels() * adfGeoTransform[5]);

  const double eastX =  adfGeoTransform[0] + (widthInPixels()  * adfGeoTransform[1]);
  const double northY = adfGeoTransform[3];
  return {westX, southY, eastX, northY};
}

ctb::CRSBounds Dataset::bounds(const OGRSpatialReference& targetSrs) const
{

  auto [westX, southY, eastX, northY] = bounds().data();
  const auto data_srs = srs();
  if (targetSrs.IsSame(&data_srs))
    return {westX, southY, eastX, northY};

  // We need to transform the bounds to the target SRS
  // this might involve warping, i.e. some of the edges can be arcs.
  // therefore we want to walk the perimiter and get min/max from there.
  // a resolution of 2000 samples per border should give a good enough approximation.
  std::vector<double> x;
  std::vector<double> y;
  auto addCoordinate = [&](double xv, double yv) { x.emplace_back(xv); y.emplace_back(yv); };

  const auto deltaX = (eastX - westX) / 2000.0;
  if (deltaX <= 0.0)
    throw Exception("west coordinate > east coordinate. This is not supported.");
  for (double s = westX; s < eastX; s += deltaX) {
    addCoordinate(s, southY);
    addCoordinate(s, northY);
  }
  const auto deltaY = (northY - southY) / 2000.0;
  if (deltaY <= 0.0)
    throw Exception("south coordinate > north coordinate. This is not supported.");
  for (double s = southY; s < northY; s += deltaY) {
    addCoordinate(westX, s);
    addCoordinate(eastX, s);
  }
  // don't wanna miss out the max/max edge vertex
  addCoordinate(eastX, northY);

  const auto transformer = util::srsTransformation(srs(), targetSrs);
  if (! transformer->Transform(int(x.size()), x.data(), y.data())) {
    throw std::string("Could not transform dataset bounds to target SRS");
  }

  assert(!x.empty());
  assert(!y.empty());
  const double target_minX = *std::min_element(x.begin(), x.end());
  const double target_maxX = *std::max_element(x.begin(), x.end());
  const double target_minY = *std::min_element(y.begin(), y.end());
  const double target_maxY = *std::max_element(y.begin(), y.end());
  return {target_minX, target_minY, target_maxX, target_maxY};
}

OGRSpatialReference Dataset::srs() const
{
  const char *srcWKT = m_gdal_dataset->GetProjectionRef();
  if (!strlen(srcWKT))
    throw Exception("The source dataset does not have a spatial reference system assigned");
  auto srs = OGRSpatialReference(srcWKT);
  srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
  return srs;
}

ctb::i_pixel Dataset::widthInPixels() const
{
  return ctb::i_pixel(m_gdal_dataset->GetRasterXSize());
}

ctb::i_pixel Dataset::heightInPixels() const
{
  return ctb::i_pixel(m_gdal_dataset->GetRasterYSize());
}

double Dataset::widthInPixels(const ctb::CRSBounds& bounds, const OGRSpatialReference& bounds_srs) const
{
  return bounds.getWidth() / pixelWidthIn(bounds_srs);
}

double Dataset::heightInPixels(const ctb::CRSBounds& bounds, const OGRSpatialReference& bounds_srs) const
{
  return bounds.getHeight() / pixelHeightIn(bounds_srs);
}

unsigned Dataset::n_bands() const
{
  const auto n = m_gdal_dataset->GetRasterCount();
  assert(n >= 0);
  return unsigned(n);
}

GDALDataset* Dataset::gdalDataset()
{
  return m_gdal_dataset.get();
}

double Dataset::gridResolution(const OGRSpatialReference& target_srs) const
{
  return std::min(pixelWidthIn(target_srs), pixelHeightIn(target_srs));
}

double Dataset::pixelWidthIn(const OGRSpatialReference& target_srs) const
{
  const auto b0 = bounds();
  const auto b1 = util::nonExactBoundsTransform(b0, srs(), target_srs);
  return b1.getWidth() / widthInPixels();
}

double Dataset::pixelHeightIn(const OGRSpatialReference& target_srs) const
{
  const auto b = util::nonExactBoundsTransform(bounds(), srs(), target_srs);
  return b.getHeight() / heightInPixels();
}
