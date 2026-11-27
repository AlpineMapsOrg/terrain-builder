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
#include <string_view>

#include "error/Result.h"
#include "store/Metadata.h"

namespace octree::storage {

inline constexpr std::string_view metadata_file_name = "octree.storemeta";

// Reads the metadata of the octree storage at `base_path`.
[[nodiscard]]
alp::Result<store::Metadata, store::ReadMetadataError> read_metadata(const std::filesystem::path& base_path);

// Writes the metadata of the octree storage at `base_path`.
[[nodiscard]]
alp::Result<void, store::WriteMetadataError> write_metadata(const std::filesystem::path& base_path, const store::Metadata& metadata);

} // namespace octree::storage
