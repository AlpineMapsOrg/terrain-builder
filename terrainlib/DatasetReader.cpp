#include "DatasetReader.h"
#include "HeightData.h"


DatasetReader::DatasetReader(const std::shared_ptr<Dataset>& dataset, const OGRSpatialReference& targetSRS)
{

}

HeightData DatasetReader::read(const ctb::CRSBounds& bounds, unsigned width, unsigned height) const
{
  return HeightData(width, height);
}
