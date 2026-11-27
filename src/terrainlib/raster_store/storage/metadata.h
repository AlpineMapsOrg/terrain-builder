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

#include <cstdint>
#include <filesystem>
#include <string_view>

#include "error/Result.h"
#include "raster_store/pixel.h"
#include "store/Metadata.h"

namespace raster_store::storage {

inline constexpr std::string_view metadata_file_name = "raster_store.metadata";

// What a raster storage records once at creation: the ids every storage records, and the shape and meaning of its tiles.
struct Metadata {
    store::Metadata ids;
    // The side length in pixels of a stored tile, halo included.
    uint32_t stored_tile_size;
    // The side length in pixels of a tile's interior.
    uint32_t nominal_tile_size;
    // The number of halo pixels on each side of a tile's interior.
    uint32_t halo_width;
    pixel::Mapping value_mapping;

    bool operator==(const Metadata&) const = default;
};

// Checks that `metadata` follows the store's rules for its ids, that its sizes form a tile and that its value mapping is known.
[[nodiscard]]
alp::Result<void, store::InvalidMetadata> validate(const Metadata& metadata);

// Reads the metadata of the raster storage at `base_path`.
[[nodiscard]]
alp::Result<Metadata, store::ReadMetadataError> read_metadata(const std::filesystem::path& base_path);

// Writes the metadata of the raster storage at `base_path`.
[[nodiscard]]
alp::Result<void, store::WriteMetadataError> write_metadata(const std::filesystem::path& base_path, const Metadata& metadata);

} // namespace raster_store::storage
