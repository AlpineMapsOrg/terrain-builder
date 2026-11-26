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

#include <cstdint>
#include <filesystem>
#include <system_error>

#include <fmt/format.h>
#include <libassert/assert.hpp>

#include "error/Context.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "fmt_impls.h"
#include "io/FileError.h"

namespace io::filesystem {

// Whether something exists at `path`, following symlinks.
[[nodiscard]]
inline alp::Result<bool, FileError> exists(const std::filesystem::path &path) {
    std::error_code code;
    const std::filesystem::file_status status = std::filesystem::status(path, code);
    if (status.type() == std::filesystem::file_type::none) {
        return alp::make_error(FileError(code, path), "reading a file status");
    }
    return std::filesystem::exists(status);
}

// Whether `path` names a directory; false if nothing exists at `path`.
[[nodiscard]]
inline alp::Result<bool, FileError> is_directory(const std::filesystem::path &path) {
    std::error_code code;
    const std::filesystem::file_status status = std::filesystem::status(path, code);
    if (status.type() == std::filesystem::file_type::none) {
        return alp::make_error(FileError(code, path), "reading a file status");
    }
    return std::filesystem::is_directory(status);
}

// Whether `path` names a regular file; false if nothing exists at `path`.
[[nodiscard]]
inline alp::Result<bool, FileError> is_regular_file(const std::filesystem::path &path) {
    std::error_code code;
    const std::filesystem::file_status status = std::filesystem::status(path, code);
    if (status.type() == std::filesystem::file_type::none) {
        return alp::make_error(FileError(code, path), "reading a file status");
    }
    return std::filesystem::is_regular_file(status);
}

// Whether `path` itself is a symlink, without following it; false if nothing exists at `path`.
[[nodiscard]]
inline alp::Result<bool, FileError> is_symlink(const std::filesystem::path &path) {
    std::error_code code;
    const std::filesystem::file_status status = std::filesystem::symlink_status(path, code);
    if (status.type() == std::filesystem::file_type::none) {
        return alp::make_error(FileError(code, path), "reading a file status");
    }
    return std::filesystem::is_symlink(status);
}

// Whether `path` names a character device; false if nothing exists at `path`.
[[nodiscard]]
inline alp::Result<bool, FileError> is_character_file(const std::filesystem::path &path) {
    std::error_code code;
    const std::filesystem::file_status status = std::filesystem::status(path, code);
    if (status.type() == std::filesystem::file_type::none) {
        return alp::make_error(FileError(code, path), "reading a file status");
    }
    return std::filesystem::is_character_file(status);
}

// The size in bytes of the file at `path`.
[[nodiscard]]
inline alp::Result<uint64_t, FileError> file_size(const std::filesystem::path &path) {
    std::error_code code;
    const uint64_t size = std::filesystem::file_size(path, code);
    if (code) {
        return alp::make_error(FileError(code, path), "reading the file size");
    }
    return size;
}

// Truncates or zero-extends the file at `path` to `size` bytes.
[[nodiscard]]
inline alp::Result<void, FileError> resize_file(const std::filesystem::path &path, const uint64_t size) {
    std::error_code code;
    std::filesystem::resize_file(path, size, code);
    if (code) {
        return alp::make_error(FileError(code, path), "resizing the file");
    }
    return {};
}

// Whether `path` names an empty file or an empty directory.
[[nodiscard]]
inline alp::Result<bool, FileError> is_empty(const std::filesystem::path &path) {
    std::error_code code;
    const bool empty = std::filesystem::is_empty(path, code);
    if (code) {
        return alp::make_error(FileError(code, path), "checking whether it is empty");
    }
    return empty;
}

// Creates the directory at `path`, whose parent must exist; true if it was created.
[[nodiscard]]
inline alp::Result<bool, FileError> create_directory(const std::filesystem::path &path) {
    std::error_code code;
    const bool created = std::filesystem::create_directory(path, code);
    if (code) {
        return alp::make_error(FileError(code, path), "creating a directory");
    }
    return created;
}

// Creates the directory at `path` and its missing parents; true if any directory was created.
[[nodiscard]]
inline alp::Result<bool, FileError> create_directories(const std::filesystem::path &path) {
    std::error_code code;
    const bool created = std::filesystem::create_directories(path, code);
    if (code) {
        return alp::make_error(FileError(code, path), "creating directories");
    }
    return created;
}

// Removes the file or empty directory at `path`; true if there was one to remove.
[[nodiscard]]
inline alp::Result<bool, FileError> remove(const std::filesystem::path &path) {
    // The return value only says whether there was something to remove, so a failure shows in the code alone.
    std::error_code code;
    const bool removed = std::filesystem::remove(path, code);
    if (code) {
        return alp::make_error(FileError(code, path), "removing the file or empty directory");
    }
    return removed;
}

// Removes whatever is at `path` with everything below it; the number of files and directories removed.
[[nodiscard]]
inline alp::Result<uint64_t, FileError> remove_all(const std::filesystem::path &path) {
    std::error_code code;
    const uint64_t removed_count = std::filesystem::remove_all(path, code);
    if (code) {
        return alp::make_error(FileError(code, path), "removing it with everything below it");
    }
    return removed_count;
}

// Creates a hard link at `target` to the file at `source`.
[[nodiscard]]
inline alp::Result<void, FileError> create_hard_link(const std::filesystem::path &source, const std::filesystem::path &target) {
    std::error_code code;
    std::filesystem::create_hard_link(source, target, code);
    if (code) {
        return alp::make_error(FileError(code, target), fmt::format("linking to {}", source));
    }
    return {};
}

// How a copy treats a file that already exists at its destination.
enum class CopyMode {
    // Fails if anything already exists at the destination.
    CreateNew,
    // Writes into the existing file, following a symlink at the destination, and drops its old bytes.
    Overwrite,
};

namespace detail {

// The standard library's options for a copy in `mode`.
constexpr std::filesystem::copy_options to_copy_options(const CopyMode mode) {
    switch (mode) {
    case CopyMode::CreateNew:
        return std::filesystem::copy_options::none;
    case CopyMode::Overwrite:
        return std::filesystem::copy_options::overwrite_existing;
    }
    UNREACHABLE("invalid copy mode", mode);
}

} // namespace detail

// Copies the file at `source` to `destination`, treating a file already there as `mode` says.
[[nodiscard]]
inline alp::Result<void, FileError> copy_file(const std::filesystem::path &source, const std::filesystem::path &destination, const CopyMode mode = CopyMode::CreateNew) {
    std::error_code code;
    std::filesystem::copy_file(source, destination, detail::to_copy_options(mode), code);
    if (code) {
        return alp::make_error(FileError(code, destination), fmt::format("copying from {}", source));
    }
    return {};
}

// Creates a symlink at `link` that points to `target`, which need not exist.
[[nodiscard]]
inline alp::Result<void, FileError> create_symlink(const std::filesystem::path &target, const std::filesystem::path &link) {
    std::error_code code;
    std::filesystem::create_symlink(target, link, code);
    if (code) {
        return alp::make_error(FileError(code, link), fmt::format("linking to {}", target));
    }
    return {};
}

// Renames `source` to `destination`, replacing anything already at `destination`.
[[nodiscard]]
inline alp::Result<void, FileError> rename(const std::filesystem::path &source, const std::filesystem::path &destination) {
    std::error_code code;
    std::filesystem::rename(source, destination, code);
    if (code) {
        return alp::make_error(FileError(code, destination), fmt::format("renaming from {}", source));
    }
    return {};
}

// Whether `first` and `second` name the same file, following symlinks; fails if either does not exist.
[[nodiscard]]
inline alp::Result<bool, FileError> equivalent(const std::filesystem::path &first, const std::filesystem::path &second) {
    std::error_code code;
    const bool is_equivalent = std::filesystem::equivalent(first, second, code);
    if (code) {
        return alp::make_error(FileError(code, first), fmt::format("comparing with {}", second));
    }
    return is_equivalent;
}

// The canonical form of `path`, which must exist: absolute, with symlinks, dot and dot-dot resolved.
[[nodiscard]]
inline alp::Result<std::filesystem::path, FileError> canonicalize(const std::filesystem::path &path) {
    std::error_code code;
    std::filesystem::path canonical_path = std::filesystem::canonical(path, code);
    if (code) {
        return alp::make_error(FileError(code, path), "resolving the canonical path");
    }
    return canonical_path;
}

// The absolute form of `path`, relative to the current directory.
[[nodiscard]]
inline alp::Result<std::filesystem::path, FileError> absolute(const std::filesystem::path &path) {
    std::error_code code;
    std::filesystem::path absolute_path = std::filesystem::absolute(path, code);
    if (code) {
        return alp::make_error(FileError(code, path), "making the path absolute");
    }
    return absolute_path;
}

} // namespace io::filesystem
