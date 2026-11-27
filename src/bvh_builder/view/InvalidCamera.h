#pragma once

#include <string>

#include <fmt/format.h>

#include "view/CameraOptions.h"

namespace view {

// Camera options that describe no view.
struct InvalidCamera {
    // What rules a view out.
    std::string reason;
    // The options that were rejected.
    CameraOptions options;

    bool operator==(const InvalidCamera &) const = default;
};

}

template <>
struct fmt::formatter<view::InvalidCamera> {
    constexpr fmt::format_parse_context::iterator parse(fmt::format_parse_context &context) {
        return context.begin();
    }

    fmt::format_context::iterator format(const view::InvalidCamera &error, fmt::format_context &context) const {
        return fmt::format_to(context.out(), "the camera has no view: {}", error.reason);
    }
};
