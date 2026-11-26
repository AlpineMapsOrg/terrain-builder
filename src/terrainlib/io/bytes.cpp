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

#include <algorithm>
#include <fstream>
#include <limits>
#include <system_error>

#include <fmt/format.h>

#include "cli_support/log.h"
#include "error/Context.h"
#include "error/log.h"
#include "error/try.h"
#include "error/utils.h"
#include "fmt_impls.h"
#include "io/alloc.h"
#include "io/bytes.h"
#include "io/filesystem.h"
#include "io/filesystem_utils.h"
#include "path_utils.h"

namespace io::bytes {

namespace {

// Makes the error for a failed stream on `path`, which reports no cause.
FileError make_stream_error(const std::filesystem::path &path) {
    // A stream reports no reason for a failure, so we return a general io error.
    return FileError(std::make_error_code(std::errc::io_error), path);
}

} // namespace

alp::Result<void, WriteError> write(const span bytes, const std::filesystem::path &path, const WriteOptions options) {
    LOG_TRACE("Writing bytes to path {}", path);
    constexpr auto open_context = "opening for writing";

    if (options.make_dirs) {
        ALP_TRY_CONTEXT(filesystem::create_parent_directories(path), fmt::format("opening {} for writing", path));
    }

    if (options.mode == WriteMode::Replace) {
        // Writes and syncs <path>.tmp, then renames it over path, so path holds the old or the new bytes even after a crash.
        const auto replace_context = alp::make_context("replacing {}", path);
        const std::filesystem::path temporary_path = paths::with_added_extension(path, "tmp");
        ALP_TRY_CONTEXT(bytes::write(bytes, temporary_path, {.mode = WriteMode::Overwrite, .make_dirs = false}), replace_context);

        // Once written, the temporary file is ours, so a failure from here on removes it.
        auto finish_result = alp::run_with_context(replace_context, [&]() -> alp::Result<void, WriteError> {
            ALP_TRY(filesystem::sync_file(temporary_path));
            ALP_TRY(filesystem::rename(temporary_path, path));
            return {};
        });
        if (!finish_result.has_value()) {
            LOG_WARN_IF_ERROR(filesystem::remove(temporary_path), "cleaning up after a failed replace");
        }
        return finish_result;
    }

    const std::ios::openmode open_mode = std::ios::binary | (options.mode == WriteMode::CreateNew ? std::ios::noreplace : std::ios::trunc);
    std::ofstream file(path, open_mode);
    if (!file.is_open()) {
        if (options.mode == WriteMode::CreateNew) {
            ALP_TRY_CONTEXT(const bool is_symlink, filesystem::is_symlink(path), open_context);
            ALP_TRY_CONTEXT(const bool exists, filesystem::exists(path), open_context);
            if (is_symlink || exists) {
                return alp::make_error(FileError(std::make_error_code(std::errc::file_exists), path), open_context);
            }
        }
        return alp::make_error(make_stream_error(path), open_context);
    }

    file.write(reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    if (file.fail()) {
        return alp::make_error(make_stream_error(path), "writing bytes");
    }
    file.flush();
    if (file.fail()) {
        return alp::make_error(make_stream_error(path), "flushing bytes");
    }
    file.close();
    if (file.fail()) {
        return alp::make_error(make_stream_error(path), "closing after writing");
    }

    return {};
}

alp::Result<vector, ReadError> read(const std::filesystem::path &path) {
    return bytes::read_prefix(path, std::numeric_limits<size_t>::max());
}

alp::Result<vector, ReadError> read_prefix(const std::filesystem::path &path, const size_t max_size) {
    LOG_TRACE("Reading bytes from path {}", path);
    constexpr auto open_context = "opening for reading";
    constexpr auto read_context = "reading bytes";

    // A stream cannot tell why it fails to open, so the cause of a missing or unreachable file comes from this check first.
    ALP_TRY_CONTEXT(filesystem::file_size(path), open_context);

    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        return alp::make_error(make_stream_error(path), open_context);
    }

    // The size comes from the open file, so it matches its bytes even if the path is replaced meanwhile.
    const std::streamsize size = file.tellg();
    if (size < 0) {
        return alp::make_error(make_stream_error(path), read_context);
    }
    const size_t read_size = std::min(static_cast<size_t>(size), max_size);
    ALP_TRY_CONTEXT(vector buffer, try_make_vector<uint8_t>(read_size), read_context);
    file.seekg(0);
    file.read(reinterpret_cast<char *>(buffer.data()), static_cast<std::streamsize>(buffer.size()));
    if (file.fail()) {
        return alp::make_error(make_stream_error(path), read_context);
    }

    return buffer;
}

} // namespace io::bytes
