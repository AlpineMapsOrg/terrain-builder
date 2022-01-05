#ifndef UTIL_H
#define UTIL_H

#include <memory>

#include <ogr_spatialref.h>

#include "Exception.h"
#include "ctb/types.hpp"

namespace util {

inline std::unique_ptr<OGRCoordinateTransformation> srsTransformation(const OGRSpatialReference& source, const OGRSpatialReference& targetSrs) {
  const auto data_srs = source;
  auto transformer = std::unique_ptr<OGRCoordinateTransformation>(OGRCreateCoordinateTransformation(&data_srs, &targetSrs));
  if (!transformer)
    throw Exception("Couldn't create SRS transformation");
  return transformer;
}

// this transform is non exact, because we are only transforming the corner vertices. however, due to projection warping, a rectangle can become an trapezoid with curved edges.
inline ctb::CRSBounds nonExactBoundsTransform(const ctb::CRSBounds& bounds, const OGRSpatialReference& sourceSrs, const OGRSpatialReference& targetSrs) {
  const auto transform = srsTransformation(sourceSrs, targetSrs);
  std::array xes = {bounds.getMinX(), bounds.getMaxX()};
  std::array yes = {bounds.getMinY(), bounds.getMaxY()};
  if (!transform->Transform(2, xes.data(), yes.data()))
    throw Exception("nonExactBoundsTransform failed");
  return {ctb::CRSPoint(xes[0], yes[0]), ctb::CRSPoint(xes[1], yes[1])};
}

}

#endif // UTIL_H
