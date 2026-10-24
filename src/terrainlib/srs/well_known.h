#pragma once

#include "error/unwrap.h"
#include "srs/System.h"

namespace srs {

// WGS84's geocentric system, EPSG:4978.
[[nodiscard]]
inline System ecef() {
    return alp::throwing_unwrap(from_epsg(4978));
}

// Web Mercator, EPSG:3857.
[[nodiscard]]
inline System webmercator() {
    return alp::throwing_unwrap(from_epsg(3857));
}

// WGS84's geographic system, EPSG:4326.
[[nodiscard]]
inline System wgs84() {
    return alp::throwing_unwrap(from_epsg(4326));
}

// MGI's geographic system, EPSG:4312.
[[nodiscard]]
inline System mgi() {
    return alp::throwing_unwrap(from_epsg(4312));
}

}
