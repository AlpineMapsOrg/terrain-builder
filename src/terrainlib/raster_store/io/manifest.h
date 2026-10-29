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

#include <cstdint>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include "io/envelope.h"
#include "raster_store/StoreTraits.h"
#include "raster_store/Tile.h"
#include "raster_store/pixel.h"
#include "store/IndexFormat.h"

namespace raster_store::io::manifest {

inline constexpr std::string_view metadata_file_name = "raster_store.metadata";
inline constexpr std::string_view index_file_name = "raster_store.index";

namespace detail::v1 {

    static_assert(sizeof(unsigned) == sizeof(std::uint32_t), "raster index v1 requires 32-bit tile ID components");

    struct Metadata {
        std::string layout_id;
        std::string payload_type;
        std::string codec_selector;
        unsigned stored_tile_size = default_tile_side;
        unsigned nominal_tile_size = default_tile_side;
        unsigned halo_width = 0;
        pixel::Mapping value_mapping = pixel::Mapping::Linear;
    };

    struct IndexEntry {
        radix::tile::Id id;
        store::NodeStatus::Value status;
    };

    struct HashEntry {
        radix::tile::Id id;
        store::NodeDataHash hash;
    };

    struct Hierarchy {
        std::vector<IndexEntry> entries;
        std::vector<HashEntry> hashes;
    };

} // namespace detail::v1

namespace detail::v2 {

    struct Metadata {
        std::string layout_id;
        std::string payload_type;
        std::string codec_selector;
        unsigned stored_tile_size = default_tile_side;
        unsigned nominal_tile_size = default_tile_side;
        unsigned halo_width = 0;
        pixel::Mapping value_mapping = pixel::Mapping::Linear;
        pixel::Placement placement = pixel::Placement::Area;
        // One for vertex tiles storing the eastern column and southern row that close their grid.
        unsigned overlap = 0;

        static Metadata from_previous(v1::Metadata previous)
        {
            return {
                .layout_id = std::move(previous.layout_id),
                .payload_type = std::move(previous.payload_type),
                .codec_selector = std::move(previous.codec_selector),
                .stored_tile_size = previous.stored_tile_size,
                .nominal_tile_size = previous.nominal_tile_size,
                .halo_width = previous.halo_width,
                .value_mapping = previous.value_mapping,
                .placement = pixel::Placement::Area,
                .overlap = 0,
            };
        }
    };

} // namespace detail::v2

using MetadataSchema = ::io::envelope::
    PayloadSchema<"raster_store.Metadata", ::io::envelope::Version<1, detail::v1::Metadata>, ::io::envelope::Version<2, detail::v2::Metadata>>;
using IndexSchema = ::io::envelope::PayloadSchema<"raster_store.Index", ::io::envelope::Version<1, detail::v1::Hierarchy>>;
using Metadata = MetadataSchema::latest_type;

Expected<void> validate(const Metadata& metadata);
Expected<Metadata> read_metadata(const std::filesystem::path& base_path);
Expected<void> write_metadata(const Metadata& metadata, const std::filesystem::path& base_path);
Expected<store::IndexContents<StoreTraits>> read_hierarchy(const std::filesystem::path& index_path);
Expected<store::IndexContents<StoreTraits>> decode_index(const detail::v1::Hierarchy& encoded);
detail::v1::Hierarchy encode_index(const store::Index<StoreTraits>& index, const store::Key2Hash_Map<StoreTraits>& hashes);
Expected<store::IndexMetadata<StoreTraits>> read_index_file(const std::filesystem::path& index_path);
Expected<void> write_index_file(const std::filesystem::path& index_path, const store::IndexMetadata<StoreTraits>& metadata);
store::IndexFormat<StoreTraits> index_format();

} // namespace raster_store::io::manifest
