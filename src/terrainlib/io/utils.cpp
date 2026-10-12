/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2025 Martin Braunsperger
 * Copyright (C) 2025 Adam Celarek-Litofcenko
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

#include "io/utils.h"

#include <cerrno>
#include <cstdio>

#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#endif

namespace io::utils {

Expected<void> create_parent_directories(const std::filesystem::path& path)
{
    std::error_code error;
    const std::filesystem::path parent_path = std::filesystem::absolute(path, error).parent_path();
    if (error) {
        return Error::fail(Error::Code::Io, "resolve parent directory for", path, error);
    }
    std::filesystem::create_directories(parent_path, error);
    if (error) {
        return Error::fail(error == std::errc::file_exists ? Error::Code::AlreadyExists : Error::Code::Io,
            "create parent directories for", path, error);
    }
    return {};
}

Expected<void> rename_without_replacement(const std::filesystem::path& source, const std::filesystem::path& destination)
{
#ifdef __linux__
    if (::renameat2(AT_FDCWD, source.c_str(), AT_FDCWD, destination.c_str(), RENAME_NOREPLACE) == 0) {
        return {};
    }
    const std::error_code error(errno, std::generic_category());
    if (error == std::errc::no_such_file_or_directory) {
        return Error::fail(Error::Code::NotFound, "rename without replacing destination", source, destination, error);
    }
    return Error::fail(error == std::errc::file_exists ? Error::Code::AlreadyExists : Error::Code::Io,
        "rename without replacing destination", source, destination, error);
#else
    return Error::fail(Error::Code::Unsupported, "rename without replacement is not implemented on this platform",
        source, destination, std::make_error_code(std::errc::operation_not_supported));
#endif
}

Expected<void> sync_file(const std::filesystem::path& path)
{
#ifdef __linux__
    const int file = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (file < 0) {
        return Error::fail(Error::Code::Io, "open file for syncing", path, std::error_code(errno, std::generic_category()));
    }
    const int synced = ::fsync(file);
    const std::error_code error(errno, std::generic_category());
    ::close(file);
    if (synced != 0) {
        return Error::fail(Error::Code::Io, "sync file", path, error);
    }
    return {};
#else
    return Error::fail(
        Error::Code::Unsupported, "syncing files is not implemented on this platform", path, std::make_error_code(std::errc::operation_not_supported));
#endif
}

} // namespace io::utils
