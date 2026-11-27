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
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include <fmt/format.h>
#include <fmt/std.h>

#include "common_error_codes.h"
#include "error/Error.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/compression.h"
#include "raster_store/Tile.h"
#include "raster_store/io/TileCodec.h"
#include "raster_store/attribution/table_file.h"
#include "raster_store/pixel.h"
#include "raster_store/storage/Index.h"
#include "raster_store/storage/index_format.h"
#include "raster_store/storage/metadata.h"
#include "store/Metadata.h"
#include "store/Draft.h"
#include "store/Storage.h"
#include "store/create.h"
#include "store/path_layout.h"
#include "store/open.h"
#include "store/publish.h"

namespace raster_store::storage {

template <typename PixelType>
using Storage = store::Storage<Index, Tile<PixelType>>;
template <typename PixelType>
using Draft = store::Draft<Index, Tile<PixelType>>;

// The payload class of raster storages whose pixels are PixelType.
template <typename PixelType>
inline const std::string payload_class = pixel::identifier<PixelType>();

using OpenError = alp::Error<store::ReadMetadataError, attribution::ReadTableError, store::ResolveError, store::OpenError>;
// Opens the raster storage at `path`, if its state is one `accept` allows.
template <typename PixelType>
[[nodiscard]]
alp::Result<Storage<PixelType>, OpenError> open(const std::filesystem::path& path, const store::Accept accept = store::Accept::Published) {
    const auto context = alp::make_context("opening the raster storage of {} at {}", payload_class<PixelType>, path);

    // Read the metadata and the attribution table
    ALP_TRY_CONTEXT(const Metadata metadata, read_metadata(path), context);
    // The tiles' attribution indices refer to this table, so a storage without a valid one is not opened.
    ALP_TRY_CONTEXT(attribution::read_table(path), context);

    // Open the storage
    const auto resolve_codec = [&metadata](const std::string_view id) { return io::tile_codec::resolve<PixelType>(id, metadata.stored_tile_size); };
    auto open_result = store::open<Index, Tile<PixelType>>(path, index_format(), metadata.ids, payload_class<PixelType>, resolve_codec, accept);
    ALP_TRY_CONTEXT(auto storage, std::move(open_result), context);

    return storage;
}

struct CreateOptions {
    uint32_t nominal_tile_size = default_tile_side;
    uint32_t halo_width = 0;
    pixel::Placement placement = pixel::Placement::Area;
    // One for vertex tiles, which store the eastern column and southern row that close their grid.
    uint32_t overlap = 0;
    // The pixel type's default mapping if not given.
    std::optional<pixel::Mapping> value_mapping = std::nullopt;
    std::string codec_id = std::string(io::tile_codec::amort_id);
    ::io::compression::Algorithm compression_algorithm = ::io::compression::Algorithm::ZstdDefault;
    // The storage whose attribution table the new storage gets a copy of; without one, it uses the table found beside it.
    std::optional<std::filesystem::path> copy_attribution_from = std::nullopt;
};

struct InvalidOptions : alp::MessageCode<"the options do not describe a valid raster storage"> {};

using CreateError = alp::Error<InvalidOptions, Unsupported, attribution::ReadTableError, store::CreateError, attribution::CopyTableError, store::WriteMetadataError>;
// Makes an empty raster draft at `<final_path>.part` with the tiles `options` describes.
template <typename PixelType>
[[nodiscard]]
alp::Result<Draft<PixelType>, CreateError> create(const std::filesystem::path& final_path, const CreateOptions& options = {}) {
    const auto context = alp::make_context("creating the raster storage of {} at {}", payload_class<PixelType>, final_path);

    // Check the options and the attribution table
    const store::path_layout::Layout<radix::tile::Id> layout = index_format().default_layout();
    const Metadata metadata{
        .ids = {
            .layout_id = std::string(layout.id),
            .payload_class = payload_class<PixelType>,
            .codec_id = options.codec_id,
        },
        // validate rejects a sum that wraps.
        .stored_tile_size = options.nominal_tile_size + options.overlap + 2 * options.halo_width,
        .nominal_tile_size = options.nominal_tile_size,
        .halo_width = options.halo_width,
        .value_mapping = options.value_mapping.value_or(pixel::default_mapping<PixelType>),
        .placement = options.placement,
        .overlap = options.overlap,
    };
    ALP_TRY_CONTEXT(alp::wrap_error(validate(metadata), InvalidOptions{}), context);
    ALP_TRY_CONTEXT(auto codec, io::tile_codec::resolve<PixelType>(metadata.ids.codec_id, metadata.stored_tile_size, options.compression_algorithm), context);
    // The tiles' attribution indices refer to this table, so nothing is created without a valid one.
    const std::filesystem::path table_source = options.copy_attribution_from.value_or(final_path);
    ALP_TRY_CONTEXT(attribution::read_table(table_source), context);

    // Create the storage
    auto create_result = store::create<Index, Tile<PixelType>>(final_path, index_format(), layout, std::move(codec));
    ALP_TRY_CONTEXT(auto draft, std::move(create_result), context);
    if (options.copy_attribution_from.has_value()) {
        ALP_TRY_CONTEXT(attribution::copy_table(*options.copy_attribution_from, draft.base_path()), context);
    }
    ALP_TRY_CONTEXT(write_metadata(draft.base_path(), metadata), context);

    return draft;
}

// Saves the draft's index, renames `<path>.part` to `<path>` and removes the input record.
template <typename PixelType>
[[nodiscard]]
alp::Result<void, store::PublishError> publish(Draft<PixelType> draft) {
    ALP_TRY(store::publish(std::move(draft)));
    return {};
}

} // namespace raster_store::storage
