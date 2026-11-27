#pragma once

#include <numbers>

namespace srs {

// The quantity a unit measures.
enum class Quantity {
    Length,
    Angle,
};

// A unit of measure.
struct Unit {
    Quantity quantity;
    // Metres for a length, radians for an angle.
    double si_size;

    bool operator==(const Unit &) const = default;
};

namespace units {

inline constexpr Unit metre{Quantity::Length, 1.0};
inline constexpr Unit radian{Quantity::Angle, 1.0};
inline constexpr Unit degree{Quantity::Angle, std::numbers::pi / 180};

}

}
