/*****************************************************************************
 * AlpineMaps.org
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

#include <glm/glm.hpp>
#include <radix/tile.h>

#include "error/Error.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "gis/srs/System.h"
#include "gis/srs/error.h"

namespace gis::srs {

// The system is not geographic.
struct NotGeographic : alp::MessageCode<"the spatial reference system is not geographic"> {};

// Why the longitudes of bounds cannot be wrapped in a reference system.
using WrapLongitudesError = alp::Error<NotGeographic, UnsupportedAxisOrder>;

// Shifts the longitudes of `bounds` by whole circles so that they start within half a circle of the longitude of `centre`, splitting the bounds in two if they then reach past the end of that range.
// Fails if `reference` is not geographic or has no angular longitude among its first two data axes.
[[nodiscard]]
alp::Result<std::vector<radix::tile::SrsBounds>, WrapLongitudesError> wrap_longitudes(const System &reference, const radix::tile::SrsBounds &bounds, const glm::dvec2 &centre);

}
