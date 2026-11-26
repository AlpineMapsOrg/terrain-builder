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

#include "common_error_codes.h"
#include "error/Error.h"
#include "error/Result.h"
#include "io/FileError.h"
#include "io/bytes.h"
#include "io/json/Value.h"
#include "io/json/serde.h"

namespace io::json {

// An error from writing a JSON file.
using WriteError = alp::Error<Unsupported, FileError>;

// Writes `value` to `path` as JSON laid out as `format`.
// Fails with Unsupported for a value to_string rejects, or with a FileError if the file cannot be written.
[[nodiscard]]
alp::Result<void, WriteError> write(const Value &value, const std::filesystem::path &path, Format format = Format::Compact, bytes::WriteOptions options = {});

} // namespace io::json
