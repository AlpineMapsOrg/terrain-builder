#pragma once

#include <string>

#include "srs/Unit.h"

namespace srs {

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
