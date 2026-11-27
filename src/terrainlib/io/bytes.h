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

#pragma once

#include <cstdint>
#include <filesystem>
#include <span>
#include <vector>

#include "common_error_codes.h"
#include "error/Error.h"
#include "error/Result.h"
#include "io/FileError.h"
#include "io/WriteMode.h"
#include "io/alloc.h"
#include "io/limits.h"

namespace io::bytes {

using vector = std::vector<uint8_t>;
using span = std::span<const uint8_t>;
using span_mut = std::span<uint8_t>;

using ReadError = alp::Error<FileError, AllocError>;

// How `write` treats an existing file and missing parent directories.
struct WriteOptions {
    WriteMode mode = default_write_mode;
    bool make_dirs = default_make_dirs;
};

using WriteError = alp::Error<FileError>;
// Writes `bytes` to the file at `path`.
[[nodiscard]]
alp::Result<void, WriteError> write(span bytes, const std::filesystem::path &path, WriteOptions options = {});

// Reads the whole file at `path`.
// Fails if the file is larger than `max_buffer_size`.
[[nodiscard]]
alp::Result<vector, ReadError> read(const std::filesystem::path &path);

// Reads the first `max_size` bytes of the file at `path`, or the whole file if it is shorter.
// Fails if the bytes it reads would be more than `max_buffer_size`.
[[nodiscard]]
alp::Result<vector, ReadError> read_prefix(const std::filesystem::path &path, size_t max_size);

// An error from reading or writing a file of bytes.
using Error = alp::Error<ReadError, WriteError>;

} // namespace io::bytes
