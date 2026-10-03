#pragma once

#include <filesystem>

#include <fmt/format.h>

namespace io {

// A directory that had to be empty but holds something.
struct DirectoryNotEmpty {
    std::filesystem::path path;

    bool operator==(const DirectoryNotEmpty &) const = default;
};

} // namespace io

template <>
struct fmt::formatter<io::DirectoryNotEmpty> {
    constexpr fmt::format_parse_context::iterator parse(fmt::format_parse_context &context) {
        return context.begin();
    }

    fmt::format_context::iterator format(const io::DirectoryNotEmpty &error, fmt::format_context &context) const {
        return fmt::format_to(context.out(), "directory {} is not empty", error.path.string());
    }
};
