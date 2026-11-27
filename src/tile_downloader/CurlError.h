/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Martin Braunsperger
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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

#include <string>
#include <string_view>
#include <utility>

#include <curl/curl.h>
#include <libassert/assert.hpp>

namespace tile_downloader {

// A failed cURL call, with the code and message cURL reported for it.
class CurlError {
public:
    explicit CurlError(const CURLcode code, std::string message) : _code(code), _message(std::move(message)) {
        DEBUG_ASSERT(code != CURLE_OK);
    }

    // Whether both errors have the same code.
    bool operator==(const CurlError &other) const {
        return this->_code == other._code;
    }

    // Whether this error has the code `code`.
    bool operator==(const CURLcode code) const {
        return this->_code == code;
    }

    // The code cURL reported.
    CURLcode code() const {
        return this->_code;
    }

    // The message cURL recorded.
    const std::string &message() const {
        return this->_message;
    }

private:
    CURLcode _code;
    std::string _message;
};

inline std::string_view format_as(const CurlError &error) {
    return error.message();
}

} // namespace tile_downloader
