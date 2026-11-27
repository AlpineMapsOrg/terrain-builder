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

#include "error/Error.h"
#include "io/alloc.h"
#include "io/mat.h"

namespace mesh {

// The part of an encoded mesh's geometry that could not be decoded.
enum class GeometryDecodeError {
    IncompatibleData,
    Positions,
    Uvs,
    Triangles
};

inline std::string_view format_as(const GeometryDecodeError error) {
    switch (error) {
    case GeometryDecodeError::IncompatibleData:
        return "the data is incompatible";
    case GeometryDecodeError::Positions:
        return "decoding the positions failed";
    case GeometryDecodeError::Uvs:
        return "decoding the UVs failed";
    case GeometryDecodeError::Triangles:
        return "decoding the triangles failed";
    default:
        return "an unknown decoding failure";
    }
}

// An error from encoding a mesh.
using EncodeError = alp::Error<::io::mat::EncodeError>;

// An error from decoding a mesh.
using DecodeError = alp::Error<GeometryDecodeError, ::io::AllocError, ::io::mat::DecodeError>;

} // namespace mesh
