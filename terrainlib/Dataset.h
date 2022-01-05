#ifndef DATASET_H
#define DATASET_H

#include <memory>
#include <string>

#include "ctb/types.hpp"

class GDALDataset;
class OGRSpatialReference;
class OGRCoordinateTransformation;

namespace ctb {
class Grid;
}

class Dataset {
public:
  Dataset(const std::string& path);
  Dataset(GDALDataset* dataset);  // takes over ownership
  ~Dataset();

  ctb::CRSBounds bounds() const;
  ctb::CRSBounds bounds(const OGRSpatialReference& targetSrs) const;
  OGRSpatialReference srs() const;
  ctb::i_pixel widthInPixels() const;
  ctb::i_pixel heightInPixels() const;
  unsigned n_bands() const;
  GDALDataset* gdalDataset();

  double pixelWidthIn(const OGRSpatialReference& targetSrs) const;
  double pixelHeightIn(const OGRSpatialReference& targetSrs) const;

private:
  std::unique_ptr<GDALDataset> m_gdal_dataset;
};

#endif
