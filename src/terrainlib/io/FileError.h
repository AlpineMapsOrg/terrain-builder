#pragma once

#include <filesystem>
#include <system_error>
#include <utility>

#include <fmt/format.h>
#include <libassert/assert.hpp>

namespace io {

// A file or filesystem operation that failed.
class FileError {
public:
    explicit FileError(const std::error_code code, std::filesystem::path path) : _code(code), _path(std::move(path)) {
        DEBUG_ASSERT(code);
    }

    bool operator==(const FileError &) const = default;

    // Whether the code corresponds to `condition`.
    bool operator==(const std::errc condition) const {
        return this->_code == condition;
    }

    // Whether the code reports that some element of the path does not exist.
    [[nodiscard]]
    bool is_not_found() const {
        return this->_code == std::errc::no_such_file_or_directory || this->_code == std::errc::not_a_directory;
    }

    // The code the operation reported.
    const std::error_code &code() const {
        return this->_code;
    }

    // The path the operation was on.
    const std::filesystem::path &path() const {
        return this->_path;
    }

private:
    std::error_code _code;
    std::filesystem::path _path;
};

// Creates a file not found error for `path`.
[[nodiscard]]
inline FileError make_file_not_found(std::filesystem::path path) {
    return FileError(std::make_error_code(std::errc::no_such_file_or_directory), std::move(path));
}

} // namespace io

template <>
struct fmt::formatter<io::FileError> {
    constexpr fmt::format_parse_context::iterator parse(fmt::format_parse_context &context) {
        return context.begin();
    }

    fmt::format_context::iterator format(const io::FileError &error, fmt::format_context &context) const {
        return fmt::format_to(context.out(), "{}: {}", error.path().string(), error.code().message());
    }
};
