#ifndef DATASETREADER_H
#define DATASETREADER_H

#include <memory>
#include "ctb/types.hpp"

class Dataset;
class OGRSpatialReference;
class HeightData;

class DatasetReader
{
public:
  DatasetReader(const std::shared_ptr<Dataset>& dataset, const OGRSpatialReference& targetSRS);

  HeightData read(const ctb::CRSBounds& bounds, unsigned width, unsigned height) const;
};

#endif // DATASETREADER_H
