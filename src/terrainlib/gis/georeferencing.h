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

#include <string_view>

#include "error/MessageCode.h"

namespace gis {

// The kinds of georeferencing that tie a dataset's pixels to coordinates.
enum class GeoreferencingKind {
    GeoTransform,
    GcpPolynomial,
    Rpc,
    GeolocationArray
};

inline std::string_view format_as(const GeoreferencingKind kind) {
    switch (kind) {
    case GeoreferencingKind::GeoTransform:
        return "geotransform";
    case GeoreferencingKind::GcpPolynomial:
        return "GCPs";
    case GeoreferencingKind::Rpc:
        return "RPC";
    case GeoreferencingKind::GeolocationArray:
        return "geolocation arrays";
    default:
        return "an unknown kind of georeferencing";
    }
}

// The georeferencing cannot be turned into a transform.
struct InvalidGeoreferencing : alp::MessageCode<"the georeferencing cannot be turned into a transform"> {};

} // namespace gis
