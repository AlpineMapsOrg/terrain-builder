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

#include <concepts>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include <cpl_error.h>
#include <libassert/assert.hpp>

#include "error/Result.h"
#include "error/utils.h"

namespace gis {

// A failed GDAL call, with the category and message GDAL recorded for it.
class CplError {
public:
    explicit CplError(const CPLErrorNum code, std::string message) : _code(code), _message(std::move(message)) {
        DEBUG_ASSERT(code != CPLE_None);
    }

    // The error GDAL last recorded on this thread, if it recorded one.
    static std::optional<CplError> try_get_last_error() {
        const CPLErrorNum code = CPLGetLastErrorNo();
        if (code == CPLE_None) {
            return std::nullopt;
        }
        return CplError(code, CPLGetLastErrorMsg());
    }

    // The error GDAL last recorded on this thread, or an application-defined one if it recorded none.
    static CplError get_last_error() {
        return try_get_last_error().value_or(CplError(CPLE_AppDefined, "GDAL recorded no error"));
    }

    // Whether both errors have the same category.
    bool operator==(const CplError &other) const {
        return this->_code == other._code;
    }

    // Whether this error has the category `code`.
    bool operator==(const CPLErrorNum code) const {
        return this->_code == code;
    }

    // The category GDAL assigned.
    CPLErrorNum code() const {
        return this->_code;
    }

    // The message GDAL recorded.
    const std::string &message() const {
        return this->_message;
    }

private:
    CPLErrorNum _code;
    std::string _message;
};

inline std::string_view format_as(const CplError &error) {
    return error.message();
}

// Fails with the error GDAL last recorded on this thread if `code` is CE_Failure or CE_Fatal, otherwise succeeds.
[[nodiscard]]
inline alp::Result<void, CplError> to_result(const CPLErr code) {
    if (code == CE_Failure || code == CE_Fatal) {
        return alp::make_error(CplError::get_last_error());
    }
    return {};
}

// Runs `call` with a cleared GDAL error state and returns its result with the error GDAL recorded during it, if it recorded one.
template <std::invocable Call>
std::pair<std::invoke_result_t<Call>, std::optional<CplError>> capture_error(Call call) {
    CPLErrorReset();
    auto value = call();
    return {std::move(value), CplError::try_get_last_error()};
}

} // namespace gis
