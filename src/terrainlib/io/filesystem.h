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

#include <concepts>
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

// Whether `path` is an empty directory; false if nothing exists at `path`.
[[nodiscard]]
inline alp::Result<bool, FileError> is_empty_directory(const std::filesystem::path &path) {
    ALP_TRY(const bool directory, filesystem::is_directory(path));
    if (!directory) {
        return false;
    }
    return filesystem::is_empty(path);
}

// Whether nothing exists at `path`, or `path` names an empty directory.
[[nodiscard]]
inline alp::Result<bool, FileError> is_missing_or_empty_directory(const std::filesystem::path &path) {
    ALP_TRY(const bool exists, filesystem::exists(path));
    if (!exists) {
        return true;
    }
    return filesystem::is_empty_directory(path);
}

// Calls `visit` with each regular file and symlink below `path`, stopping early once `visit` returns false.
template <std::predicate<const std::filesystem::path &> Visit>
[[nodiscard]]
alp::Result<void, FileError> visit_files_recursively(const std::filesystem::path &path, Visit &&visit) {
    const auto context = alp::make_context("listing files at {}", path);

    std::error_code code;
    std::filesystem::recursive_directory_iterator iterator(path, code);
    if (code) {
        return alp::make_error(FileError(code, path), context);
    }

    const std::filesystem::recursive_directory_iterator end;
    while (iterator != end) {
        const std::filesystem::directory_entry &entry = *iterator;
        const std::filesystem::file_status status = entry.symlink_status(code);
        if (status.type() == std::filesystem::file_type::none) {
            return alp::make_error(FileError(code, entry.path()), context);
        }
        if (std::filesystem::is_regular_file(status) || std::filesystem::is_symlink(status)) {
            if (!visit(entry.path())) {
                return {};
            }
        }

        iterator.increment(code);
        if (code) {
            return alp::make_error(FileError(code, path), context);
        }
    }
    return {};
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

// Creates an empty directory at `path`, along with its missing parents.
[[nodiscard]]
inline alp::Result<void, FileError> create_empty_directory(const std::filesystem::path &path) {
    constexpr auto context = "creating an empty directory";

    ALP_TRY_CONTEXT(const bool exists, filesystem::exists(path), context);
    if (!exists) {
        ALP_TRY_CONTEXT(filesystem::create_directories(path), context);
        return {};
    }

    ALP_TRY_CONTEXT(const bool is_directory, filesystem::is_directory(path), context);
    if (!is_directory) {
        return alp::make_error(FileError(std::make_error_code(std::errc::not_a_directory), path), context);
    }

    ALP_TRY_CONTEXT(const bool empty, filesystem::is_empty(path), context);
    if (!empty) {
        return alp::make_error(FileError(std::make_error_code(std::errc::directory_not_empty), path), context);
    }
    return {};
}

// Creates the missing parent directories of `path`; true if any directory was created.
[[nodiscard]]
inline alp::Result<bool, FileError> create_parent_directories(const std::filesystem::path &path) {
    const std::filesystem::path parent_path = path.parent_path();
    if (parent_path.empty()) {
        return false;
    }
    return filesystem::create_directories(parent_path);
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

// Renames `source` to `destination`, failing if anything is already at `destination`.
[[nodiscard]]
alp::Result<void, FileError> rename_no_replace(const std::filesystem::path &source, const std::filesystem::path &destination);

// Flushes the data and metadata of the file at `path` to the storage device.
[[nodiscard]]
alp::Result<void, FileError> sync_file(const std::filesystem::path &path);

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

// Whether `first` and `second` are on the same filesystem, following symlinks; fails if either does not exist.
[[nodiscard]]
alp::Result<bool, FileError> on_same_filesystem(const std::filesystem::path &first, const std::filesystem::path &second);

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

// The canonical form of `path`, or `path` as given if it cannot be made canonical.
[[nodiscard]]
inline std::filesystem::path canonicalize_best_effort(const std::filesystem::path &path) {
    std::error_code code;
    std::filesystem::path canonical = std::filesystem::weakly_canonical(path, code);
    if (code) {
        return path;
    }
    return canonical;
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
