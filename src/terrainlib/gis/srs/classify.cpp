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

#include "gis/srs/classify.h"

#include <algorithm>
#include <functional>
#include <ranges>
#include <vector>

#include <libassert/assert.hpp>

#include "optional_utils.h"
#include "range_utils.h"
#include "string_utils.h"

namespace gis::srs {

namespace {

bool is_horizontal(const Axis &axis) {
    switch (axis.direction) {
    case AxisDirection::East:
    case AxisDirection::West:
    case AxisDirection::North:
    case AxisDirection::South:
        return true;
    case AxisDirection::Up:
    case AxisDirection::Down:
    case AxisDirection::Other:
        return false;
    }
    UNREACHABLE("invalid direction", axis.direction);
}

bool is_vertical(const Axis &axis) {
    switch (axis.direction) {
    case AxisDirection::Up:
    case AxisDirection::Down:
        return true;
    case AxisDirection::East:
    case AxisDirection::West:
    case AxisDirection::North:
    case AxisDirection::South:
    case AxisDirection::Other:
        return false;
    }
    UNREACHABLE("invalid direction", axis.direction);
}

// The unit all of `units` are approximately in, if there are any and they agree.
template <std::ranges::forward_range Units>
std::optional<Unit> find_common_unit(Units &&units) {
    if (std::ranges::empty(units)) {
        return std::nullopt;
    }
    const Unit first = *std::ranges::begin(units);
    if (!std::ranges::all_of(units, [&](const Unit &unit) { return unit.is_approx(first); })) {
        return std::nullopt;
    }
    return first;
}

}

bool has_axis_order(const System &srs, const std::initializer_list<AxisDirection> directions) {
    return starts_with(srs.axes(), directions, std::ranges::equal_to{}, &Axis::direction);
}

bool is_east_north_axis_order(const System &srs) {
    const std::vector<Axis> axes = srs.axes();
    if (axes.size() < 2) {
        return false;
    }
    // A polar projection's axes both point north or both south, along meridians, so their names tell which is which.
    // This mirrors GDAL's isNorthEastAxisOrder, which checks authority order for north, then east: https://github.com/OSGeo/gdal/blob/v3.10.3/ogr/ogrspatialreference.cpp#L419-L443
    const bool both_north_or_both_south = (axes[0].direction == AxisDirection::North && axes[1].direction == AxisDirection::North) || (axes[0].direction == AxisDirection::South && axes[1].direction == AxisDirection::South);
    if (both_north_or_both_south) {
        return starts_with_case_insensitive(axes[0].name, "easting") && starts_with_case_insensitive(axes[1].name, "northing");
    }
    return axes[0].direction == AxisDirection::East && axes[1].direction == AxisDirection::North;
}

std::optional<size_t> find_axis_index(const System &srs, const AxisDirection direction) {
    const std::vector<Axis> axes = srs.axes();
    return index_of(axes | std::views::transform(&Axis::direction), direction);
}

std::optional<LongitudeLatitudeAxes> longitude_latitude_axes(const System &srs) {
    if (srs.kind() != Kind::Geographic) {
        return std::nullopt;
    }

    const std::vector<Axis> axes = srs.axes();
    const auto directions = axes | std::views::transform(&Axis::direction);
    const std::optional<size_t> longitude_opt = index_of(directions, AxisDirection::East).or_else([&] { return index_of(directions, AxisDirection::West); });
    const std::optional<size_t> latitude_opt = index_of(directions, AxisDirection::North).or_else([&] { return index_of(directions, AxisDirection::South); });
    if (!longitude_opt.has_value() || !latitude_opt.has_value()) {
        return std::nullopt;
    }
    const size_t longitude_index = *longitude_opt;
    const size_t latitude_index = *latitude_opt;
    const IndexedAxis longitude{.index = longitude_index, .axis = axes[longitude_index]};
    const IndexedAxis latitude{.index = latitude_index, .axis = axes[latitude_index]};

    const bool are_angles = longitude.axis.unit.quantity == Quantity::Angle && latitude.axis.unit.quantity == Quantity::Angle;
    if (!are_angles) {
        return std::nullopt;
    }
    return LongitudeLatitudeAxes{.longitude = longitude, .latitude = latitude};
}

std::optional<Unit> horizontal_unit(const System &srs) {
    const std::vector<Axis> axes = srs.axes();
    return find_common_unit(axes | std::views::filter(is_horizontal) | std::views::transform(&Axis::unit));
}

bool has_horizontal_axes_in(const System &srs, const Unit &unit) {
    return alp::transform_or(horizontal_unit(srs), false, [&](const Unit &horizontal) { return horizontal.is_approx(unit); });
}

std::optional<Unit> vertical_unit(const System &srs) {
    const std::vector<Axis> axes = srs.axes();
    return find_common_unit(axes | std::views::filter(is_vertical) | std::views::transform(&Axis::unit));
}

bool has_vertical_axes_in(const System &srs, const Unit &unit) {
    return alp::transform_or(vertical_unit(srs), false, [&](const Unit &vertical) { return vertical.is_approx(unit); });
}

}
