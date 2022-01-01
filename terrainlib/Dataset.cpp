#include "Dataset.h"

#include <algorithm>
#include <cassert>
#include <stdexcept>

#include <gdal_priv.h>

#include "ctb/Grid.hpp"
#include "tntn/logging.h"
#include "tntn/gdal_init.h"

Dataset::Dataset(const std::string& path)
{
  tntn::initialize_gdal_once();
  m_gdal_dataset.reset(static_cast<GDALDataset *>(GDALOpen(path.c_str(), GA_ReadOnly)));
  if (!m_gdal_dataset) {
    TNTN_LOG_FATAL("Couldn't open dataset {}.\n", path);
    throw std::runtime_error("");
  }
}

std::unique_ptr<OGRCoordinateTransformation> Dataset::srsTransformation(const OGRSpatialReference& targetSrs) const
{
  const auto data_srs = srs();
  auto transformer = std::unique_ptr<OGRCoordinateTransformation>(OGRCreateCoordinateTransformation(&data_srs, &targetSrs));
  if (!transformer)
    throw std::runtime_error("Couldn't create SRS transformation");
  return transformer;
}

Dataset::~Dataset() = default;

ctb::CRSBounds Dataset::bounds(const OGRSpatialReference& targetSrs) const
{
  std::array<double, 6> adfGeoTransform = {};
  if (m_gdal_dataset->GetGeoTransform(adfGeoTransform.data()) != CE_None)
    throw std::runtime_error("Could not get transformation information from source dataset");

  // https://gdal.org/user/raster_data_model.html
  // gdal has a row/column raster format, where row 0 is the top most row.
  // an affine transform is used to convert row/column into the datasets SRS.
  // computing bounds is going first from row/column to dataset SRS and then to target SRS

  // we don't support sheering or rotation for now
  if (adfGeoTransform[2] != 0.0 || adfGeoTransform[4] != 0.0)
    throw std::runtime_error("Dataset geo transform contains sheering or rotation. This is not supported!");

  const double width = m_gdal_dataset->GetRasterXSize();
  const double height = m_gdal_dataset->GetRasterYSize();
//  const double westX =  adfGeoTransform[0] + (0.5            * adfGeoTransform[1]);
//  const double southY = adfGeoTransform[3] + ((height - 0.5) * adfGeoTransform[5]);

//  const double eastX =  adfGeoTransform[0] + ((width - 0.5)  * adfGeoTransform[1]);
//  const double northY = adfGeoTransform[3] + (0.5            * adfGeoTransform[5]);

  const double westX =  adfGeoTransform[0];
  const double southY = adfGeoTransform[3] + (height * adfGeoTransform[5]);

  const double eastX =  adfGeoTransform[0] + (width  * adfGeoTransform[1]);
  const double northY = adfGeoTransform[3];
  auto bound_in_data_crs = ctb::CRSBounds(westX, southY, eastX, northY);
  const auto data_srs = srs();
  if (targetSrs.IsSame(&data_srs))
    return bound_in_data_crs;

  // We need to transform the bounds to the target SRS
  // this might involve warping, i.e. some of the edges can be arcs.
  // therefore we want to walk the perimiter and get min/max from there.
  // a resolution of 2000 samples per border should give a good enough approximation.
  std::vector<double> x;
  std::vector<double> y;
  auto addCoordinate = [&](double xv, double yv) { x.emplace_back(xv); y.emplace_back(yv); };

  const auto deltaX = (eastX - westX) / 2000.0;
  if (deltaX <= 0.0)
    throw std::runtime_error("west coordinate > east coordinate. This is not supported.");
  for (double s = westX; s < eastX; s += deltaX) {
    addCoordinate(s, southY);
    addCoordinate(s, northY);
  }
  const auto deltaY = (northY - southY) / 2000.0;
  if (deltaY <= 0.0)
    throw std::runtime_error("south coordinate > north coordinate. This is not supported.");
  for (double s = southY; s < northY; s += deltaY) {
    addCoordinate(westX, s);
    addCoordinate(eastX, s);
  }
  // don't wanna miss out the max/max edge vertex
  addCoordinate(eastX, northY);

  const auto transformer = srsTransformation(targetSrs);
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
    throw std::runtime_error("The source dataset does not have a spatial reference system assigned");
  auto srs = OGRSpatialReference(srcWKT);
  srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
  return srs;
}
