/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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

#include "store/snapshot.h"

#include <cerrno>
#include <system_error>

#include <sys/stat.h>

#include "cli_support/log.h"
#include "io/utils.h"
#include "store/inputs.h"

namespace store::snapshot {

std::filesystem::path partial_path(const std::filesystem::path& final_path)
{
    auto path = final_path.lexically_normal();
    path += ".part";
    return path;
}

bool is_partial(const std::filesystem::path& path)
{
    auto normalized = path.lexically_normal();
    if (!normalized.has_filename()) {
        normalized = normalized.parent_path();
    }
    return normalized.extension() == ".part";
}

Expected<void> require_absent(const std::filesystem::path& path)
{
    std::error_code error;
    const auto status = std::filesystem::symlink_status(path, error);
    if (error && error != std::errc::no_such_file_or_directory) {
        return Error::fail(Error::Code::Io, "inspect snapshot destination", path, error);
    }
    if (std::filesystem::exists(status)) {
        return Error::fail(Error::Code::AlreadyExists, "create snapshot at", path);
    }
    return {};
}

Expected<std::filesystem::path> create_partial(const std::filesystem::path& final_path)
{
    const auto destination = final_path.lexically_normal();
    if (destination.filename().empty() || destination.filename() == "." || destination.filename() == ".." || is_partial(destination)) {
        return Error::fail(Error::Code::InvalidInput, "creation requires a final snapshot path without a .part suffix", final_path);
    }
    if (auto absent = require_absent(destination); !absent) {
        return Error::propagate(std::move(absent));
    }
    const auto partial = partial_path(destination);
    if (auto absent = require_absent(partial); !absent) {
        return Error::propagate(std::move(absent));
    }
    std::error_code error;
    if (!destination.parent_path().empty()) {
        std::filesystem::create_directories(destination.parent_path(), error);
        if (error) {
            return Error::fail(Error::Code::Io, "create snapshot parent directory", destination.parent_path(), error);
        }
    }
    if (!std::filesystem::create_directory(partial, error)) {
        return Error::fail(error ? Error::Code::Io : Error::Code::AlreadyExists, "create incomplete snapshot", partial, error);
    }
    return partial;
}

Expected<void> publish(const std::filesystem::path& partial_path)
{
    const auto partial = partial_path.lexically_normal();
    if (!is_partial(partial)) {
        return Error::fail(Error::Code::InvalidInput, "publication requires an incomplete .part snapshot", partial_path);
    }
    auto destination = partial;
    destination.replace_extension();
    if (auto absent = require_absent(destination); !absent) {
        return absent;
    }
    // The record stays until the rename succeeded, so a .part that failed to publish remains a cache.
    auto renamed = ::io::utils::rename_without_replacement(partial, destination);
    if (!renamed) {
        return Error::propagate(std::move(renamed), "publish snapshot");
    }
    // A leftover record is harmless, as a published snapshot is never accepted as a cache.
    const auto inputs_path = destination / inputs::file_name;
    std::error_code error;
    std::filesystem::remove(inputs_path, error);
    if (error) {
        LOG_WARN("Failed to remove the input record {} from the published snapshot: {}", inputs_path, error.message());
    }
    return {};
}

Expected<void> check_link_filesystem(const std::filesystem::path& source, const std::filesystem::path& output)
{
    struct stat source_status {};
    if (::stat(source.c_str(), &source_status) != 0) {
        return Error::fail(Error::Code::Io, "inspect link source filesystem", source, std::error_code(errno, std::generic_category()));
    }
    auto parent = std::filesystem::absolute(output).parent_path();
    struct stat output_status {};
    while (::stat(parent.c_str(), &output_status) != 0) {
        if (errno != ENOENT || parent == parent.parent_path()) {
            return Error::fail(Error::Code::Io, "inspect output filesystem", parent, std::error_code(errno, std::generic_category()));
        }
        parent = parent.parent_path();
    }
    if (source_status.st_dev != output_status.st_dev) {
        return Error::fail(Error::Code::Unsupported, "hard links require the source and the output to be on the same filesystem", source);
    }
    return {};
}

} // namespace store::snapshot
