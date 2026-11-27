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

#include <fmt/format.h>
#include <jxl/encode.h>
#include <libassert/assert.hpp>

namespace io::image {

// A failure libjxl's encoder reported.
class JxlError {
public:
    explicit JxlError(const JxlEncoderError code) : _code(code) {
        DEBUG_ASSERT(code != JXL_ENC_ERR_OK);
    }

    // The failure `encoder` reported last.
    static JxlError get_last_error(JxlEncoder &encoder) {
        return JxlError(JxlEncoderGetError(&encoder));
    }

    bool operator==(const JxlError &) const = default;

    // The code libjxl reported.
    JxlEncoderError code() const {
        return this->_code;
    }

private:
    JxlEncoderError _code;
};

// What the code means, as libjxl describes it.
inline std::string format_as(const JxlError error) {
    switch (error.code()) {
    case JXL_ENC_ERR_GENERIC:
        return "generic encoder error due to unspecified cause";
    case JXL_ENC_ERR_OOM:
        return "out of memory";
    case JXL_ENC_ERR_JBRD:
        return "JPEG bitstream reconstruction data could not be represented";
    case JXL_ENC_ERR_BAD_INPUT:
        return "input is invalid";
    case JXL_ENC_ERR_NOT_SUPPORTED:
        return "the encoder does not support this";
    case JXL_ENC_ERR_API_USAGE:
        return "the encoder API is used in an incorrect way";
    default:
        return fmt::format("libjxl encoder error {}", fmt::underlying(error.code()));
    }
}

} // namespace io::image
