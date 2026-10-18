#pragma once

#include <string_view>

#include <libassert/assert.hpp>

namespace alp {

// An error code of which no value can exist.
struct Infallible {
    Infallible() = delete;
};

inline std::string_view format_as(Infallible) {
    UNREACHABLE();
}

} // namespace alp
