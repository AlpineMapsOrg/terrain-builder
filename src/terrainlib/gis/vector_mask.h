/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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

#pragma once

#include <CGAL/Multipolygon_with_holes_2.h>
#include <CGAL/Polygon_set_2.h>
#include <ogr_spatialref.h>

#include "Error.h"
#include "mesh/cgal.h"

namespace gis {
class Dataset;
}

namespace gis::vector_mask {

using Kernel = cgal::kernel::epeck::Kernel;
using Point2 = Kernel::Point_2;
using Polygon2 = CGAL::Polygon_2<Kernel>;
using PolygonWithHoles2 = CGAL::Polygon_with_holes_2<Kernel>;
using PolygonSet2 = CGAL::Polygon_set_2<Kernel>;
using MultipolygonWithHoles2 = CGAL::Multipolygon_with_holes_2<Kernel>;

struct ReferencedPolygonMask {
    MultipolygonWithHoles2 polygons;
    OGRSpatialReference srs;
};

/// Loads the polygons of all features in all layers, without simplification or reprojection.
/// Outer boundaries are counterclockwise, holes clockwise. A dataset without a spatial
/// reference is assumed to be WGS84. Fails for layers with different spatial references,
/// non-polygonal geometries, rings that are not simple, read failures and empty masks.
Expected<ReferencedPolygonMask> load(Dataset& dataset);

/// Reprojects the polygons into target_srs. Straight edges in the source SRS are curved in the
/// target SRS; edges are subdivided until the result deviates from the curved edges by at most
/// tolerance_m metres. Fails if a ring is no longer simple after reprojection.
/// An error passed in is returned unchanged.
Expected<ReferencedPolygonMask> transform(const Expected<ReferencedPolygonMask>& polygons, const OGRSpatialReference& target_srs, double tolerance_m);

/// Removes vertices while the result deviates from the original by less than tolerance_m metres.
/// Preserves the topology across all rings: rings neither start to intersect nor change their
/// nesting. An error passed in is returned unchanged.
Expected<ReferencedPolygonMask> simplify(const Expected<ReferencedPolygonMask>& polygons, double tolerance_m);

} // namespace gis::vector_mask
