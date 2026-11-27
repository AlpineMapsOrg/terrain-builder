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

#include "io/filesystem.h"

#include <cerrno>
#include <cstdio>
#include <system_error>

#ifdef __linux__
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#include "containers/UniqueResource.h"

namespace io::filesystem {

#ifdef __linux__
namespace {

// Closes a POSIX file descriptor.
struct CloseFileDescriptor {
    void operator()(const int descriptor) const noexcept {
        ::close(descriptor);
    }
};

} // namespace
#endif

alp::Result<void, FileError> rename_no_replace(const std::filesystem::path &source, const std::filesystem::path &destination) {
    const auto context = alp::make_context("renaming from {} without replacement", source);
#ifdef __linux__
    if (::renameat2(AT_FDCWD, source.c_str(), AT_FDCWD, destination.c_str(), RENAME_NOREPLACE) != 0) {
        return alp::make_error(FileError(std::error_code(errno, std::generic_category()), destination), context);
    }
    return {};
#else
    return alp::make_error(FileError(std::make_error_code(std::errc::operation_not_supported), destination), context);
#endif
}

alp::Result<void, FileError> sync_file(const std::filesystem::path &path) {
    constexpr auto context = "flushing the file to the storage device";
#ifdef __linux__
    const int raw_descriptor = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (raw_descriptor < 0) {
        return alp::make_error(FileError(std::error_code(errno, std::generic_category()), path), context);
    }
    const UniqueResource<int, CloseFileDescriptor> descriptor(raw_descriptor);
    // The error is made before the descriptor closes, so closing cannot overwrite errno.
    if (::fsync(descriptor.get()) != 0) {
        return alp::make_error(FileError(std::error_code(errno, std::generic_category()), path), context);
    }
    return {};
#else
    return alp::make_error(FileError(std::make_error_code(std::errc::operation_not_supported), path), context);
#endif
}


alp::Result<bool, FileError> on_same_filesystem(const std::filesystem::path &first, const std::filesystem::path &second) {
    constexpr auto context = "reading a file status";
#ifdef __linux__
    struct stat first_status {};
    if (::stat(first.c_str(), &first_status) != 0) {
        return alp::make_error(FileError(std::error_code(errno, std::generic_category()), first), context);
    }
    struct stat second_status {};
    if (::stat(second.c_str(), &second_status) != 0) {
        return alp::make_error(FileError(std::error_code(errno, std::generic_category()), second), context);
    }
    return first_status.st_dev == second_status.st_dev;
#else
    return alp::make_error(FileError(std::errc::operation_not_supported, first), context);
#endif
}

} // namespace io::filesystem
