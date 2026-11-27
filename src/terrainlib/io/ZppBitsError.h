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
#include <system_error>

#include <libassert/assert.hpp>

namespace io {

// A value zpp_bits could not serialize or deserialize.
class ZppBitsError {
public:
    explicit ZppBitsError(const std::errc code) : _code(code) {
        DEBUG_ASSERT(code != std::errc{});
    }

    bool operator==(const ZppBitsError &) const = default;

    // The code zpp_bits reported.
    std::errc code() const {
        return this->_code;
    }

private:
    friend std::string_view format_as(ZppBitsError error);

    std::errc _code;
};

// What the code means for serializing or deserializing, as zpp_bits documents it.
inline std::string_view format_as(const ZppBitsError error) {
    switch (error._code) {
    case std::errc::result_out_of_range:
        return "the buffer is too short";
    case std::errc::no_buffer_space:
        return "the buffer would grow beyond the allocation limits";
    case std::errc::message_size:
        return "the message size is beyond the allocation limits";
    case std::errc::value_too_large:
        return "a variable-length integer is beyond its representation limits";
    case std::errc::bad_message:
        return "a variant holds an unrecognized type";
    case std::errc::invalid_argument:
        return "a null pointer or a value-less variant cannot be serialized";
    case std::errc::not_supported:
        return "a remote procedure that is not listed as supported was called";
    case std::errc::protocol_error:
        return "the data is an invalid protocol message";
    default:
        return "an unknown (de)serialization failure";
    }
}

} // namespace io
