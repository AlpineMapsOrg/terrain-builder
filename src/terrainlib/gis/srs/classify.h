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

#include <cstddef>
#include <initializer_list>
#include <optional>

#include "gis/srs/Axis.h"
#include "gis/srs/System.h"
#include "gis/srs/Unit.h"

namespace gis::srs {

// Whether the data axes of `srs` start with axes pointing in `directions`, in that order.
[[nodiscard]]
bool has_axis_order(const System &srs, std::initializer_list<AxisDirection> directions);

// Whether the first two data axes of `srs` are its easting or longitude, then its northing or latitude.
[[nodiscard]]
bool is_east_north_axis_order(const System &srs);

// The index of the first data axis of `srs` that points in `direction`, if one does.
[[nodiscard]]
std::optional<size_t> find_axis_index(const System &srs, AxisDirection direction);

// An axis together with its position among a system's data axes.
struct IndexedAxis {
    size_t index;
    Axis axis;
};

// The longitude and latitude among the data axes of a geographic system.
struct LongitudeLatitudeAxes {
    IndexedAxis longitude;
    IndexedAxis latitude;
};

// The data axes of `srs` that hold its longitude and latitude, if it is geographic and has both, measured as angles.
[[nodiscard]]
std::optional<LongitudeLatitudeAxes> longitude_latitude_axes(const System &srs);

// The unit the horizontal data axes of `srs` share, if it has any and they share one.
[[nodiscard]]
std::optional<Unit> horizontal_unit(const System &srs);

// Whether `srs` has horizontal data axes, all approximately in `unit`.
[[nodiscard]]
bool has_horizontal_axes_in(const System &srs, const Unit &unit);

// The unit the vertical data axes of `srs` share, if it has any and they share one.
[[nodiscard]]
std::optional<Unit> vertical_unit(const System &srs);

// Whether `srs` has vertical data axes, all approximately in `unit`.
[[nodiscard]]
bool has_vertical_axes_in(const System &srs, const Unit &unit);

}
