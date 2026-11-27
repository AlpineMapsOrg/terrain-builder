/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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

#include <fmt/format.h>
#include <fmt/std.h>

#include "common_error_codes.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "io/FileError.h"
#include "io/bytes.h"
#include "raster_store/attribution/Table.h"

namespace raster_store::attribution {

inline constexpr std::string_view file_name = "source_attribution_table.json";

struct CorruptTable : alp::MessageCode<"the attribution table is not valid"> {};

using ParseError = alp::Error<CorruptTable, Unsupported, MakeTableError>;
// The table that `json` holds: an array whose slot i describes the source with attribution index i.
[[nodiscard]]
alp::Result<Table, ParseError> parse(std::string_view json);

// No attribution table lies in a storage's directory or its two ancestor directories.
struct MissingTable {
    // The storage whose table was looked for.
    std::filesystem::path storage_path;

    bool operator==(const MissingTable&) const = default;
};

inline std::string format_as(const MissingTable& error) {
    return fmt::format("no attribution table lies in {} or its two ancestor directories", error.storage_path);
}

using FindTableError = alp::Error<::io::FileError, MissingTable>;
// The path of the attribution table for the storage at `storage_path`: the nearest of its directory, its parent and its grandparent that holds an entry named file_name, even a broken symlink.
[[nodiscard]]
alp::Result<std::filesystem::path, FindTableError> find_table(const std::filesystem::path& storage_path);

using ReadTableError = alp::Error<FindTableError, ::io::bytes::ReadError, ParseError>;
// Reads the attribution table of the storage at `storage_path`, found as find_table does.
[[nodiscard]]
alp::Result<Table, ReadTableError> read_table(const std::filesystem::path& storage_path);

using CopyTableError = alp::Error<FindTableError, ::io::FileError>;
// Copies the attribution table of the storage at `source_storage_path`, found as find_table does, into `destination_directory`; fails if that directory does not exist or already holds one.
[[nodiscard]]
alp::Result<void, CopyTableError> copy_table(const std::filesystem::path& source_storage_path, const std::filesystem::path& destination_directory);

} // namespace raster_store::attribution
