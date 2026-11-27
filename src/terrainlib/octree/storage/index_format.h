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
#include <string_view>

#include "error/Result.h"
#include "octree/Id.h"
#include "octree/storage/Index.h"
#include "store/IndexFormat.h"
#include "store/path_layout.h"

namespace octree::storage {

inline constexpr std::string_view index_file_name = "octree.storeindex";

// Reads the index of the octree storage at `base_path`.
[[nodiscard]]
alp::Result<Index, store::ReadIndexError> read_index(const std::filesystem::path& base_path);

// Writes the index of the octree storage at `base_path`.
[[nodiscard]]
alp::Result<void, store::WriteIndexError> write_index(const std::filesystem::path& base_path, const Index& index);

// The layout octree storages use unless one is chosen.
store::path_layout::Layout<Id> default_layout();

// The format of octree storages' index files, and the layouts their node files can use.
store::IndexFormat<Index> index_format();

} // namespace octree::storage
