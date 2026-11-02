#pragma once

#include <concepts>
#include <cstdint>
#include <filesystem>
#include <system_error>

#include <fmt/format.h>

#include "error/Context.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "fmt_impls.h"
#include "io/DirectoryNotEmpty.h"
#include "io/FileError.h"

namespace io::filesystem {

// Whether something exists at `path`, following symlinks.
[[nodiscard]]
inline alp::Result<bool, FileError> exists(const std::filesystem::path &path) {
    std::error_code code;
    const std::filesystem::file_status status = std::filesystem::status(path, code);
    if (status.type() == std::filesystem::file_type::none) {
        return alp::make_error(FileError(code, path), alp::make_context("reading a file status at {}", path));
    }
    return std::filesystem::exists(status);
}

// Whether `path` names a directory; false if nothing exists at `path`.
[[nodiscard]]
inline alp::Result<bool, FileError> is_directory(const std::filesystem::path &path) {
    std::error_code code;
    const std::filesystem::file_status status = std::filesystem::status(path, code);
    if (status.type() == std::filesystem::file_type::none) {
        return alp::make_error(FileError(code, path), alp::make_context("reading a file status at {}", path));
    }
    return std::filesystem::is_directory(status);
}

// Whether `path` names a regular file; false if nothing exists at `path`.
[[nodiscard]]
inline alp::Result<bool, FileError> is_regular_file(const std::filesystem::path &path) {
    std::error_code code;
    const std::filesystem::file_status status = std::filesystem::status(path, code);
    if (status.type() == std::filesystem::file_type::none) {
        return alp::make_error(FileError(code, path), alp::make_context("reading a file status at {}", path));
    }
    return std::filesystem::is_regular_file(status);
}

// Whether `path` itself is a symlink, without following it; false if nothing exists at `path`.
[[nodiscard]]
inline alp::Result<bool, FileError> is_symlink(const std::filesystem::path &path) {
    std::error_code code;
    const std::filesystem::file_status status = std::filesystem::symlink_status(path, code);
    if (status.type() == std::filesystem::file_type::none) {
        return alp::make_error(FileError(code, path), alp::make_context("reading a file status at {}", path));
    }
    return std::filesystem::is_symlink(status);
}

// Whether `path` names an empty file or an empty directory.
[[nodiscard]]
inline alp::Result<bool, FileError> is_empty(const std::filesystem::path &path) {
    std::error_code code;
    const bool empty = std::filesystem::is_empty(path, code);
    if (code) {
        return alp::make_error(FileError(code, path), alp::make_context("checking whether {} is empty", path));
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
        return alp::make_error(FileError(code, path), alp::make_context("creating a directory at {}", path));
    }
    return created;
}

// Creates the directory at `path` and its missing parents; true if any directory was created.
[[nodiscard]]
inline alp::Result<bool, FileError> create_directories(const std::filesystem::path &path) {
    std::error_code code;
    const bool created = std::filesystem::create_directories(path, code);
    if (code) {
        return alp::make_error(FileError(code, path), alp::make_context("creating directories at {}", path));
    }
    return created;
}

// Creates an empty directory at `path`, along with its missing parents.
// Fails with `DirectoryNotEmpty` when a directory at `path` already holds something.
[[nodiscard]]
inline alp::Result<void, FileError, DirectoryNotEmpty> create_empty_directory(const std::filesystem::path &path) {
    const auto context = alp::make_context("creating an empty directory at {}", path);

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
        return alp::make_error(DirectoryNotEmpty{path}, context);
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
        return alp::make_error(FileError(code, path), alp::make_context("removing {}", path));
    }
    return removed;
}

// Removes whatever is at `path` with everything below it; the number of files and directories removed.
[[nodiscard]]
inline alp::Result<uint64_t, FileError> remove_all(const std::filesystem::path &path) {
    std::error_code code;
    const uint64_t removed_count = std::filesystem::remove_all(path, code);
    if (code) {
        return alp::make_error(FileError(code, path), alp::make_context("removing everything at {}", path));
    }
    return removed_count;
}

// Creates a hard link at `target` to the file at `source`.
[[nodiscard]]
inline alp::Result<void, FileError> create_hard_link(const std::filesystem::path &source, const std::filesystem::path &target) {
    std::error_code code;
    std::filesystem::create_hard_link(source, target, code);
    if (code) {
        return alp::make_error(FileError(code, target), alp::make_context("linking {} to {}", target, source));
    }
    return {};
}

// Whether `first` and `second` name the same file, following symlinks; fails if either does not exist.
[[nodiscard]]
inline alp::Result<bool, FileError> equivalent(const std::filesystem::path &first, const std::filesystem::path &second) {
    std::error_code code;
    const bool is_equivalent = std::filesystem::equivalent(first, second, code);
    if (code) {
        return alp::make_error(FileError(code, first), alp::make_context("comparing {} with {}", first, second));
    }
    return is_equivalent;
}

// The canonical form of `path`, or `path` as given if it cannot be made canonical.
[[nodiscard]]
inline std::filesystem::path canonicalize(const std::filesystem::path &path) {
    std::error_code code;
    std::filesystem::path canonical = std::filesystem::weakly_canonical(path, code);
    if (code) {
        return path;
    }
    return canonical;
}

} // namespace io::filesystem
