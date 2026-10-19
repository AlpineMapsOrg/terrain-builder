/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2022 Adam Celarek <family name at cg tuwien ac at>
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
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>

#include <fmt/format.h>
#include <ogr_spatialref.h>

#include "error/Result.h"
#include "error/unwrap.h"
#include "error/utils.h"
#include "gdal/OgrError.h"

namespace srs {

// A spatial reference system.
using System = OGRSpatialReference;

inline std::optional<int> epsg_code(const System &srs) {
    System tmp = srs;
    tmp.AutoIdentifyEPSG();

    const char *authName = tmp.GetAuthorityName(nullptr);
    const char *authCode = tmp.GetAuthorityCode(nullptr);

    if (authName == nullptr || authCode == nullptr) {
        return std::nullopt;
    }

    if (std::string(authName) != "EPSG") {
        return std::nullopt;
    }

    return std::atoi(authCode);
}

inline std::string friendly_name(const System &srs) {
    if (const auto code = epsg_code(srs)) {
        return "EPSG:" + std::to_string(*code);
    }

    const char *name = srs.GetName();

    if (name != nullptr && *name != '\0') {
        return std::string(name);
    }

    return "unknown";
}

inline System clone(const System &srs) {
    const std::unique_ptr<System> cloned(srs.Clone());
    return std::move(*cloned);
}

[[nodiscard]]
inline alp::Result<System, gdal::OgrError> from_epsg(const uint32_t epsg) {
    System srs;
    if (const OGRErr code = srs.importFromEPSG(epsg); code != OGRERR_NONE) {
        return alp::make_unexpected(gdal::OgrError(code), fmt::format("importing reference from EPSG code {}", epsg));
    }
    srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    return srs;
}

[[nodiscard]]
inline alp::Result<System, gdal::OgrError> from_user_input(const std::string &user_input) {
    System srs;
    if (const OGRErr code = srs.SetFromUserInput(user_input.c_str()); code != OGRERR_NONE) {
        return alp::make_unexpected(gdal::OgrError(code), fmt::format("parsing a spatial reference from {}", user_input));
    }
    srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    return srs;
}

inline System ecef() {
    return alp::unwrap(from_epsg(4978));
}
inline System webmercator() {
    return alp::unwrap(from_epsg(3857));
}
inline System wgs84() {
    return alp::unwrap(from_epsg(4326));
}
inline System mgi() {
    return alp::unwrap(from_epsg(4312));
}

}
