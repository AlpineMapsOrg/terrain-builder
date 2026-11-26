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
#include <filesystem>
#include <system_error>

#include "error/Context.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/FileError.h"
#include "io/filesystem.h"

namespace io::filesystem {

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

// Renames `source` to `destination`, failing if anything is already at `destination`.
[[nodiscard]]
alp::Result<void, FileError> rename_no_replace(const std::filesystem::path &source, const std::filesystem::path &destination);

// Flushes the data and metadata of the file at `path` to the storage device.
[[nodiscard]]
alp::Result<void, FileError> sync_file(const std::filesystem::path &path);

// Whether `first` and `second` are on the same filesystem, following symlinks; fails if either does not exist.
[[nodiscard]]
alp::Result<bool, FileError> on_same_filesystem(const std::filesystem::path &first, const std::filesystem::path &second);

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

// Requires that nothing exists at `path`, not even a symlink.
[[nodiscard]]
alp::Result<void, FileError> require_absent(const std::filesystem::path &path);

// Requires that `source` and the closest existing ancestor of `output` are on the same filesystem.
[[nodiscard]]
alp::Result<void, FileError> require_same_filesystem(const std::filesystem::path &source, const std::filesystem::path &output);

} // namespace io::filesystem
