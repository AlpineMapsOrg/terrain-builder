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

#include <libassert/assert.hpp>
#include <ogr_core.h>

#include "error/Result.h"
#include "error/utils.h"

namespace gis {

// A failed OGR call, identified by the code it returned.
class OgrError {
public:
    explicit OgrError(const OGRErr code) : _code(code) {
        DEBUG_ASSERT(code != OGRERR_NONE);
    }

    bool operator==(const OgrError &) const = default;

    // Whether this error has the code `code`.
    bool operator==(const OGRErr code) const {
        return this->_code == code;
    }

    // The code the call returned.
    OGRErr code() const {
        return this->_code;
    }

private:
    OGRErr _code;
};

// What the code means, as OGR documents it.
inline std::string_view format_as(const OgrError error) {
    switch (error.code()) {
    case OGRERR_NOT_ENOUGH_DATA:
        return "not enough data to deserialize";
    case OGRERR_NOT_ENOUGH_MEMORY:
        return "not enough memory";
    case OGRERR_UNSUPPORTED_GEOMETRY_TYPE:
        return "unsupported geometry type";
    case OGRERR_UNSUPPORTED_OPERATION:
        return "unsupported operation";
    case OGRERR_CORRUPT_DATA:
        return "corrupt data";
    case OGRERR_FAILURE:
        return "failure";
    case OGRERR_UNSUPPORTED_SRS:
        return "unsupported spatial reference system";
    case OGRERR_INVALID_HANDLE:
        return "invalid handle";
    case OGRERR_NON_EXISTING_FEATURE:
        return "non-existing feature";
    default:
        return "an unknown OGR failure";
    }
}

// Fails with `code` unless it is OGRERR_NONE, otherwise succeeds.
[[nodiscard]]
inline alp::Result<void, OgrError> to_result(const OGRErr code) {
    if (code != OGRERR_NONE) {
        return alp::make_error(OgrError(code));
    }
    return {};
}

} // namespace gis
