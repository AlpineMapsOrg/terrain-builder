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

class Dataset;
using DatasetPtr = std::shared_ptr<Dataset>;

class Dataset {
public:
  Dataset(const std::string& path);
  Dataset(GDALDataset* dataset);  // takes over ownership
  ~Dataset();
  static DatasetPtr make_shared(const std::string& path);
  static DatasetPtr make(GDALDataset* dataset);  // takes over ownership

  [[nodiscard]] std::string name() const;

  [[nodiscard]] ctb::CRSBounds bounds() const;
  [[nodiscard]] ctb::CRSBounds bounds(const OGRSpatialReference& targetSrs) const;
  [[nodiscard]] OGRSpatialReference srs() const;
  [[nodiscard]] ctb::i_pixel widthInPixels() const;
  [[nodiscard]] ctb::i_pixel heightInPixels() const;
  [[nodiscard]] double widthInPixels(const ctb::CRSBounds& bounds, const OGRSpatialReference& bounds_srs) const;
  [[nodiscard]] double heightInPixels(const ctb::CRSBounds& bounds, const OGRSpatialReference& bounds_srs) const;
  [[nodiscard]] unsigned n_bands() const;
  [[nodiscard]] GDALDataset* gdalDataset();

  [[nodiscard]] double gridResolution(const OGRSpatialReference& target_srs) const;
  [[nodiscard]] double pixelWidthIn(const OGRSpatialReference& target_srs) const;
  [[nodiscard]] double pixelHeightIn(const OGRSpatialReference& target_srs) const;

private:
  std::unique_ptr<GDALDataset> m_gdal_dataset;
  std::string m_name;
};

#endif
