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

    explicit FileError(const std::errc code, std::filesystem::path path) : FileError(std::make_error_code(code), std::move(path)) {
    }

    // Whether both are on the same path with codes that mean the same, whichever category reported them.
    bool operator==(const FileError &other) const {
        return this->_code.default_error_condition() == other._code.default_error_condition() && this->_path == other._path;
    }

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
