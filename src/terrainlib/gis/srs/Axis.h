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

#include <string>

#include "gis/srs/Unit.h"

namespace gis::srs {

// The direction an axis points in.
enum class AxisDirection {
    East,
    West,
    North,
    South,
    Up,
    Down,
    Other,
};

// One axis of a system's data coordinates.
struct Axis {
    std::string name;
    AxisDirection direction;
    Unit unit;
};

}
