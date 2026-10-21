#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <ogr_spatialref.h>

#include "error/Result.h"
#include "gdal/OgrError.h"
#include "srs/Axis.h"
#include "srs/Kind.h"
#include "srs/error.h"

namespace srs {

// A geodetic spatial reference system, whose data axes are in traditional GIS order.
// It is never empty, and it must not be used from several threads at once, not even through const methods.
class System {
public:
    // The GDAL spatial reference this wraps, for passing to GDAL calls.
    [[nodiscard]]
    const OGRSpatialReference &as_gdal() const {
        return this->_srs;
    }

    // Whether coordinates in `other` mean the same as coordinates in this system.
    [[nodiscard]]
    bool is_same(const System &other) const {
        return this->_srs.IsSame(&other._srs);
    }

    // The kind of this system's coordinates.
    [[nodiscard]]
    Kind kind() const;

    // The axes of this system's data coordinates, in data order.
    [[nodiscard]]
    std::vector<Axis> axes() const;

    // Writes this system as WKT2, which from_user_input reads back.
    [[nodiscard]]
    alp::Result<std::string, gdal::OgrError> to_wkt() const;

private:
    explicit System(OGRSpatialReference srs) : _srs(std::move(srs)) {}

    friend alp::Result<System, UnsupportedSystem> from_gdal(OGRSpatialReference srs);

    OGRSpatialReference _srs;
};

// Converts the GDAL spatial reference `srs`, which must not be empty, into a system.
// Fails if it is not geodetic or its data axes are not in traditional GIS order.
[[nodiscard]]
alp::Result<System, UnsupportedSystem> from_gdal(OGRSpatialReference srs);

// Imports the system with the EPSG code `code`.
// Fails if the code is unknown or the system is not supported.
[[nodiscard]]
alp::Result<System, gdal::OgrError, UnsupportedSystem> from_epsg(uint32_t code);

// Parses a system from a definition such as "EPSG:4326", WKT or a PROJ string.
// Fails if it cannot be parsed or the system is not supported.
[[nodiscard]]
alp::Result<System, gdal::OgrError, UnsupportedSystem> from_user_input(const std::string &input);

// The EPSG code of `srs` as "EPSG:<code>", if it has one.
// A compound system without a code of its own is formatted as "EPSG:<horizontal>+<vertical>".
[[nodiscard]]
std::optional<std::string> format_epsg_code(const System &srs);

// The name of `srs`, such as "MGI / Austria Lambert", if it has one.
[[nodiscard]]
std::optional<std::string> friendly_name(const System &srs);

// Formats `srs` as "EPSG:<code> (<name>)".
std::string format_as(const System &srs);

}
