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
#include <string>
#include <string_view>

#include "error/Result.h"
#include "error/try.h"
#include "io/bytes.h"

namespace io::text {

// How `write` treats an existing file and missing parent directories.
using WriteOptions = bytes::WriteOptions;

using WriteError = bytes::WriteError;
// Writes `text` to the file at `path`.
[[nodiscard]]
inline alp::Result<void, WriteError> write(const std::string_view text, const std::filesystem::path &path, const WriteOptions options = {}) {
    const bytes::vector bytes(text.begin(), text.end());
    return bytes::write(bytes, path, options);
}

using ReadError = bytes::ReadError;
// Reads the whole file at `path` as text.
[[nodiscard]]
inline alp::Result<std::string, ReadError> read(const std::filesystem::path &path) {
    ALP_TRY(const bytes::vector bytes, bytes::read(path));
    return std::string(bytes.begin(), bytes.end());
}

} // namespace io::text
