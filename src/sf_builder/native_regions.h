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

#include <vector>

#include <radix/geometry.h>
#include <radix/tile.h>

#include "error/Error.h"
#include "error/Result.h"
#include "gis/srs/System.h"
#include "gis/srs/Transform.h"
#include "gis/srs/coverage.h"
#include "gis/srs/geographic.h"

namespace sf {

using ComputeNativeRegionsError = alp::Error<gis::srs::coverage::Error, gis::srs::WrapLongitudesError>;
// Computes the bounds in the dataset's srs that cover the ECEF bounds.
// For a geographic srs, longitudes lie in the turn centred on the native bounds, and bounds crossing its edges are split there.
[[nodiscard]]
alp::Result<std::vector<radix::tile::SrsBounds>, ComputeNativeRegionsError> compute_native_regions(
    const gis::srs::System& dataset_srs, const radix::tile::SrsBounds& native_bounds, const radix::geometry::Aabb3d& ecef_bounds);

// Cuts the regions to the native bounds, dropping those that lie outside.
[[nodiscard]]
std::vector<radix::tile::SrsBounds> clip_to_native_bounds(const std::vector<radix::tile::SrsBounds>& regions, const radix::tile::SrsBounds& native_bounds);

using IntersectsNativeBoundsError = alp::Error<gis::srs::coverage::UnsupportedSystem, gis::srs::coverage::UnsupportedBounds, gis::srs::UnsupportedTransform, gis::srs::NotGeographic>;
// Whether any part of the ECEF bounds lies within the dataset's native bounds.
// Bounds the dataset's srs cannot express lie outside.
[[nodiscard]]
alp::Result<bool, IntersectsNativeBoundsError> intersects_native_bounds(
    const gis::srs::System& dataset_srs, const radix::tile::SrsBounds& native_bounds, const radix::geometry::Aabb3d& ecef_bounds);

} // namespace sf
