#pragma once

#include <source_location>
#include <string_view>

#include <fmt/format.h>

// A source location that formats as its function, without return type and parameters, followed by its file, line and column.
struct PrunedLocation {
    std::source_location location;
};

template <>
struct fmt::formatter<PrunedLocation> : fmt::formatter<std::string_view> {
    fmt::format_context::iterator format(PrunedLocation location, fmt::format_context &context) const;
};
