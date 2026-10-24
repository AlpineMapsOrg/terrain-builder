#pragma once

namespace srs {

// An ellipsoid of revolution, which is a sphere when both semi-axes are equal.
struct Ellipsoid {
    double semi_major_metres;
    double semi_minor_metres;

    bool operator==(const Ellipsoid &) const = default;
};

}
