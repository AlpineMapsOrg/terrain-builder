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

#pragma once

#include <chrono>
#include <filesystem>

#include <expected>

#include "Error.h"

// A snapshot is produced in <final>.part and renamed to <final> once complete, so an
// existing <final> is always complete. An incomplete <final>.part can serve as a cache.
namespace store::snapshot {

inline constexpr auto checkpoint_interval = std::chrono::minutes(2);

// Lexically normal and without a trailing separator, so that "out/" names the snapshot "out".
std::filesystem::path normalized(const std::filesystem::path& path);
std::filesystem::path partial_path(const std::filesystem::path& final_path);
bool is_partial(const std::filesystem::path& path);
Expected<void> require_absent(const std::filesystem::path& path);
// Requires that neither <final> nor <final>.part exists, then creates <final>.part.
Expected<std::filesystem::path> create_partial(const std::filesystem::path& final_path);
// Renames <final>.part to <final> and removes the input record. The storage in it
// must have saved its index and must not be used afterwards.
Expected<void> publish(const std::filesystem::path& partial_path);
// Hard links require the source and the output to be on the same filesystem.
Expected<void> check_link_filesystem(const std::filesystem::path& source, const std::filesystem::path& output);

} // namespace store::snapshot
