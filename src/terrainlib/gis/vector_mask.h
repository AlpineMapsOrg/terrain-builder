/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
 * Copyright (C) 2026 Martin Braunsperger
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

#include <string_view>

#include <CGAL/Multipolygon_with_holes_2.h>
#include <CGAL/Polygon_set_2.h>

#include "error/MessageCode.h"
#include "error/Result.h"
#include "gis/srs/System.h"
#include "gis/srs/error.h"
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

// Why the features of a dataset do not form a mask.
enum class InvalidMask {
    // The layers have different spatial reference systems.
    AmbiguousSystem,
    // No feature holds a polygon.
    MissingGeometry,
    // A feature has no geometry, or one that is not a valid polygon.
    InvalidGeometry,
    // A feature has a geometry that is not a polygon.
    UnsupportedGeometry,
};

inline std::string_view format_as(const InvalidMask error) {
    switch (error) {
    case InvalidMask::AmbiguousSystem:
        return "the layers of the mask have different spatial reference systems";
    case InvalidMask::MissingGeometry:
        return "the mask contains no polygons";
    case InvalidMask::InvalidGeometry:
        return "a feature of the mask has no geometry or one that is not a valid polygon";
    case InvalidMask::UnsupportedGeometry:
        return "a feature of the mask has a geometry that is not a polygon";
    default:
        return "an unknown mask failure";
    }
}

// GDAL cannot read the features of the mask.
struct UnreadableMask : alp::MessageCode<"the features of the mask cannot be read"> {};

// The mask cannot be reprojected into the target system within the tolerance.
struct UnreprojectableMask : alp::MessageCode<"the mask cannot be reprojected into the target system within the tolerance"> {};

// The tolerance in metres cannot be expressed in the units of the mask's spatial reference system.
struct UnsupportedToleranceSrs : alp::MessageCode<"the tolerance cannot be expressed in the units of the spatial reference system"> {};

struct ReferencedPolygonMask {
    MultipolygonWithHoles2 polygons;
    gis::srs::System srs;
};

using LoadMaskError = alp::Error<InvalidMask, UnreadableMask, srs::UnsupportedAxisOrder>;
using TransformMaskError = alp::Error<UnreprojectableMask, UnsupportedToleranceSrs>;
using SimplifyMaskError = alp::Error<UnsupportedToleranceSrs>;

// Loads the polygons of all features in all layers, without simplification or reprojection.
// Outer boundaries are counterclockwise, holes clockwise; a dataset without a spatial reference system is assumed to be WGS84.
// Fails for layers with different spatial reference systems or one that does not order its data axes east, north, for missing, invalid or unsupported geometries, for read failures and for empty masks.
[[nodiscard]]
alp::Result<ReferencedPolygonMask, LoadMaskError> load(Dataset &dataset);

// Reprojects the polygons into `target_srs`, subdividing edges until the result deviates from the curved edges by at most `tolerance_m` metres.
// Fails if the mask cannot be reprojected within the tolerance, or if the tolerance cannot be expressed in the target system.
[[nodiscard]]
alp::Result<ReferencedPolygonMask, TransformMaskError> transform(const ReferencedPolygonMask &mask, const srs::System &target_srs, double tolerance_m);

// Removes vertices while the result deviates from the original by less than `tolerance_m` metres, keeping rings from crossing or changing their nesting.
// Fails if the tolerance cannot be expressed in the mask's spatial reference system.
[[nodiscard]]
alp::Result<ReferencedPolygonMask, SimplifyMaskError> simplify(const ReferencedPolygonMask &mask, double tolerance_m);

} // namespace gis::vector_mask
