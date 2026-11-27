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

#include <cstddef>
#include <expected>
#include <string_view>

#include <libassert/assert.hpp>
#include <zstd.h>
#include <zstd_errors.h>

namespace io {

// A failure Zstandard reported.
class ZstdError {
public:
    explicit ZstdError(const ZSTD_ErrorCode code) : _code(code) {
        DEBUG_ASSERT(code != ZSTD_error_no_error);
    }

    bool operator==(const ZstdError &) const = default;

    // The code Zstandard reported.
    ZSTD_ErrorCode code() const {
        return this->_code;
    }

private:
    ZSTD_ErrorCode _code;
};

// What the code means, as Zstandard describes it.
inline std::string_view format_as(const ZstdError error) {
    return ZSTD_getErrorString(error.code());
}

// The value a Zstandard function returned, or the code it reported instead.
inline std::expected<size_t, ZstdError> to_zstd_result(const size_t return_value) {
    if (ZSTD_isError(return_value)) {
        return std::unexpected(ZstdError(ZSTD_getErrorCode(return_value)));
    } else {
        return return_value;
    }
}

} // namespace io
