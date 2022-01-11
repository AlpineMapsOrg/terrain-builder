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
