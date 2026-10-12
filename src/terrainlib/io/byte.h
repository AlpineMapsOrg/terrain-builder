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

#include <cstddef>
#include <filesystem>
#include <limits>
#include <span>
#include <vector>

#include <expected>

#include "Error.h"

namespace io::byte {

using vector = std::vector<std::byte>;
using span = std::span<const std::byte>;

// Replace writes and syncs <path>.tmp, then renames it over path, so path holds the old or the
// new bytes even after a crash, and hard links to the old file keep the old bytes. Replace writes
// of the same path must not run concurrently.
enum class WriteMode { Overwrite, CreateNew, Replace };

Expected<void> write(span bytes, const std::filesystem::path& path, WriteMode mode, bool make_dirs = true);
// Reads the whole file, or its first max_size bytes if it is longer.
Expected<vector> read(const std::filesystem::path& path, std::size_t max_size = std::numeric_limits<std::size_t>::max());

}
