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

#include <string>
#include <utility>

#include <fmt/format.h>
#include <turbojpeg.h>

namespace io::image {

// A failure TurboJPEG reported.
class TurboJpegError {
public:
    // The failure `handle` reported last.
    static TurboJpegError get_last_error(const tjhandle handle) {
        return TurboJpegError(static_cast<TJERR>(tj3GetErrorCode(handle)), tj3GetErrorStr(handle));
    }

    bool operator==(const TurboJpegError &) const = default;

    // Whether TurboJPEG reported a non-fatal error, after which the image may still be corrupt.
    bool is_warning() const {
        return this->_code == TJERR_WARNING;
    }

    // What TurboJPEG said went wrong.
    const std::string &message() const {
        return this->_message;
    }

private:
    TurboJpegError(const TJERR code, std::string message) : _code(code), _message(std::move(message)) {}

    TJERR _code;
    std::string _message;
};

// What went wrong, as TurboJPEG describes it, marked as a warning if it is one.
inline std::string format_as(const TurboJpegError &error) {
    return error.is_warning() ? fmt::format("warning: {}", error.message()) : error.message();
}

} // namespace io::image
