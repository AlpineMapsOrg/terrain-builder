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

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <ogr_spatialref.h>

#include "error/Result.h"
#include "gis/OgrError.h"
#include "gis/srs/Axis.h"
#include "gis/srs/Ellipsoid.h"
#include "gis/srs/Kind.h"
#include "gis/srs/error.h"

namespace gis::srs {

// A version of the well-known text format for spatial reference systems.
enum class WktVersion {
    // GDAL's dialect of WKT1, which carries extensions such as CENTER_LONG.
    Gdal1,
    // WKT2 as published in ISO 19162:2019.
    Iso2019,
};

// A spatial reference system.
// It is never empty, and it must not be used from several threads at once.
class System {
public:
    // A system that owns the GDAL spatial reference.
    explicit System(OGRSpatialReference srs);

    // A system that shares ownership of the GDAL spatial reference.
    explicit System(std::shared_ptr<const OGRSpatialReference> srs);

    // A system that refers to the GDAL spatial reference, which must outlive it.
    explicit System(const OGRSpatialReference *srs);

    // An independent copy of this system.
    [[nodiscard]]
    System clone() const;

    // The GDAL spatial reference this wraps, for passing to GDAL calls.
    [[nodiscard]]
    const OGRSpatialReference &as_gdal() const {
        return *this->_srs;
    }

    // Whether coordinates in `other` mean the same as coordinates in this system.
    [[nodiscard]]
    bool is_same(const System &other) const {
        return this->_srs->IsSame(other._srs.get());
    }

    // The EPSG code of this system, if it has one or matches one.
    [[nodiscard]]
    std::optional<uint32_t> epsg_code() const;

    // The kind of this system's coordinates.
    [[nodiscard]]
    Kind kind() const;

    // Whether this system combines a horizontal and a vertical system.
    [[nodiscard]]
    bool is_compound() const {
        return this->_srs->IsCompound();
    }

    // The ellipsoid of this system's datum.
    [[nodiscard]]
    alp::Result<Ellipsoid, NotGeodetic> ellipsoid() const;

    // The prime meridian of this system's datum, in degrees east of Greenwich.
    [[nodiscard]]
    alp::Result<double, NotGeodetic> prime_meridian_degrees() const;

    // The axes of this system's data coordinates, in data order.
    [[nodiscard]]
    std::vector<Axis> axes() const;

    // Writes this system as WKT of `version`, which from_user_input reads back.
    [[nodiscard]]
    alp::Result<std::string, OgrError> to_wkt(WktVersion version) const;

private:
    std::shared_ptr<const OGRSpatialReference> _srs;
};

// Imports the system with the EPSG code `code`.
// Fails if the code is unknown.
[[nodiscard]]
alp::Result<System, OgrError> from_epsg(uint32_t code);

// Parses a system from a definition such as "EPSG:4326", WKT or a PROJ string.
// Fails if it cannot be parsed.
[[nodiscard]]
alp::Result<System, OgrError> from_user_input(const std::string &input);

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
