#pragma once

#include <string_view>
#include <system_error>

#include <libassert/assert.hpp>

namespace io {

// A value zpp_bits could not serialize or deserialize.
class SerdeError {
public:
    explicit SerdeError(const std::errc code) : _code(code) {
        DEBUG_ASSERT(code != std::errc{});
    }

    bool operator==(const SerdeError &) const = default;

private:
    friend std::string_view format_as(SerdeError error);

    std::errc _code;
};

// What the code means for serializing or deserializing, as zpp_bits documents it.
inline std::string_view format_as(const SerdeError error) {
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
    default:
        return "an unknown (de)serialization failure";
    }
}

} // namespace io
