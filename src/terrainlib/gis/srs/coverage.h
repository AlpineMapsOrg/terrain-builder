/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2022 Adam Celarek-Litofcenko
 * Copyright (C) 2022 Martin Braunsperger
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
#include <vector>

#include <radix/geometry.h>
#include <radix/tile.h>

#include "error/Error.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "gis/srs/System.h"
#include "gis/srs/Transform.h"
#include "gis/srs/error.h"

namespace gis::srs::coverage {

// The system combines a horizontal and a vertical system.
struct CompoundSystem : alp::MessageCode<"the spatial reference system is compound"> {};

// The system is geographic, but its angles are not in degrees.
struct AnglesNotDegrees : alp::MessageCode<"the angles of the spatial reference system are not in degrees"> {};

// The system is not geographic.
struct NotGeographic : alp::MessageCode<"the spatial reference system is not geographic"> {};

// Why coverage cannot be computed in a reference system.
using UnsupportedSystem = alp::Error<NotGeodetic, CompoundSystem, AnglesNotDegrees, UnsupportedAxisOrder>;

// Why coverage cannot be computed for bounds.
enum class UnsupportedBounds {
    // Web Mercator bounds that reach beyond the world extent.
    OutsideWebMercatorExtent,
    // Heights outside -20 km to 200 km.
    HeightOutOfRange,
    // ECEF bounds within 43 km of the earth's centre, where PROJ is erratic.
    NearEarthCentre,
};

std::string_view format_as(UnsupportedBounds error);

// An error from computing coverage.
using Error = alp::Error<UnsupportedSystem, UnsupportedBounds, TransformError>;

// WGS84 longitude/latitude bounds in degrees covering the given bounds.
// Longitudes lie in [-180, 180]; coverage crossing the antimeridian is split into two bounds.
// Reprojected coverage includes a guard band, because edge sampling can miss extrema of curved edges.
[[nodiscard]]
alp::Result<std::vector<radix::tile::SrsBounds>, Error> compute_geographic(const System &reference, const radix::tile::SrsBounds &bounds);

// Web Mercator bounds covering the given bounds within the polar latitude limits.
// Returns zero (polar-only coverage), one, or two (crossing the antimeridian) bounds.
[[nodiscard]]
alp::Result<std::vector<radix::tile::SrsBounds>, Error> compute_mercator(const System &reference, const radix::tile::SrsBounds &bounds);

// ECEF bounds covering the given bounds of a 2D SRS, whose z is ellipsoidal height.
// Goes to longitude/latitude via compute_geographic, then analytically to ECEF:
// within a longitude/latitude/height box, every ECEF coordinate is monotonic along each
// axis, except at longitudes -90, 0, 90 and the equator. Evaluating all combinations of
// the box limits and the turning values inside it gives the exact ECEF bounds of that box.
// Compound SRSes are unsupported.
[[nodiscard]]
alp::Result<radix::geometry::Aabb3d, Error> compute_ecef(const System &reference, const radix::geometry::Aabb3d &bounds);

// Bounds in a 2D SRS covering the given ECEF bounds; z is ellipsoidal height.
// Goes analytically to longitude/latitude first: longitude from the corners of the xy
// rectangle, latitude and height from a few points given by the distance to the polar axis
// and z. WGS84 and Web Mercator results are quasi exact (as tight as it gets using proj); other SRSes use GDAL's densified
// boundary transformation with a guard band. Geographic bounds are split at the
// antimeridian, so usually there is one bounds, or two when crossing it; Web Mercator
// returns none outside its latitude limits.
// Fails for compound SRSes, for bounds close to the earth's centre, where PROJ is
// erratic, and if the transformation fails.
[[nodiscard]]
alp::Result<std::vector<radix::geometry::Aabb3d>, Error> compute_from_ecef(const radix::geometry::Aabb3d &bounds, const System &reference);

// Why the longitudes of bounds cannot be wrapped in a reference system.
using WrapLongitudesError = alp::Error<NotGeographic, UnsupportedAxisOrder>;

// Shifts the longitudes of `bounds` by whole circles so that they start within half a circle of the longitude of `centre`, splitting the bounds in two if they then reach past the end of that range.
// Fails if `reference` is not geographic or has no angular longitude among its first two data axes.
// TODO: maybe move to another file?
[[nodiscard]]
alp::Result<std::vector<radix::tile::SrsBounds>, WrapLongitudesError> wrap_longitudes(const System &reference, const radix::tile::SrsBounds &bounds, const glm::dvec2 &centre);

}
