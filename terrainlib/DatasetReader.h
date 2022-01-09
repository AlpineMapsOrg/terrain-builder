#ifndef DATASETREADER_H
#define DATASETREADER_H

#include <memory>
#include <string>

#include "Image.h"
#include "ctb/types.hpp"

class Dataset;
class OGRSpatialReference;

class DatasetReader
{
public:
  DatasetReader(const std::shared_ptr<Dataset>& dataset, const OGRSpatialReference& targetSRS, unsigned band);

  HeightData read(const ctb::CRSBounds& bounds, unsigned width, unsigned height) const;
  HeightData readWithOverviews(const ctb::CRSBounds& bounds, unsigned width, unsigned height) const;

  unsigned dataset_band() const { return m_band; }
  bool isReprojecting() const { return m_requires_reprojection; }
  std::string dataset_srs_wkt() const { return m_dataset_srs_wkt; }
  std::string target_srs_wkt() const { return m_target_srs_wkt; }

protected:
  HeightData readFrom(const std::shared_ptr<Dataset>& dataset, const ctb::CRSBounds& bounds, unsigned width, unsigned height) const;

private:
  std::shared_ptr<Dataset> m_dataset;
  std::string m_dataset_srs_wkt;
  std::string m_target_srs_wkt;
  bool m_requires_reprojection;
  unsigned m_band;
};

#endif // DATASETREADER_H
