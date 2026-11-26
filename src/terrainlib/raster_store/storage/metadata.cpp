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

#include "raster_store/storage/metadata.h"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>

#include <fmt/format.h>
#include <fmt/std.h>
#include <magic_enum/magic_enum.hpp>

#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/WriteMode.h"
#include "io/envelope.h"
#include "numeric/checked.h"
#include "numeric/int_math.h"
#include "raster_store/pixel.h"
#include "store/Metadata.h"
#include "store/envelope_utils.h"

namespace raster_store::storage {

namespace {

namespace v1 {

struct Metadata {
    std::string layout_id;
    std::string payload_class;
    std::string codec_id;
    uint32_t stored_tile_size;
    uint32_t nominal_tile_size;
    uint32_t halo_width;
    pixel::Mapping value_mapping;
};

} // namespace v1

namespace v2 {

struct Metadata {
    std::string layout_id;
    std::string payload_class;
    std::string codec_id;
    uint32_t stored_tile_size;
    uint32_t nominal_tile_size;
    uint32_t halo_width;
    pixel::Mapping value_mapping;
    pixel::Placement placement;
    uint32_t overlap;

    // Reads version 1 as area placement without overlap.
    static Metadata from_previous(v1::Metadata previous) {
        return {
            .layout_id = std::move(previous.layout_id),
            .payload_class = std::move(previous.payload_class),
            .codec_id = std::move(previous.codec_id),
            .stored_tile_size = previous.stored_tile_size,
            .nominal_tile_size = previous.nominal_tile_size,
            .halo_width = previous.halo_width,
            .value_mapping = previous.value_mapping,
            .placement = pixel::Placement::Area,
            .overlap = 0,
        };
    }
};

} // namespace v2

using MetadataSchema = ::io::envelope::PayloadSchema<"raster_store.Metadata", ::io::envelope::Version<1, v1::Metadata>, ::io::envelope::Version<2, v2::Metadata>>;

} // namespace

alp::Result<void, store::InvalidMetadata> validate(const Metadata& metadata) {
    ALP_TRY(store::validate(metadata.ids));

    if (!magic_enum::enum_contains(metadata.placement)) {
        return alp::make_error(store::InvalidMetadata{}, fmt::format("the placement {} is unknown", std::to_underlying(metadata.placement)));
    }
    const uint32_t max_overlap = metadata.placement == pixel::Placement::Vertex ? 1 : 0;
    if (metadata.overlap > max_overlap) {
        return alp::make_error(store::InvalidMetadata{}, fmt::format("an overlap of {} does not fit {} placement", metadata.overlap, magic_enum::enum_name(metadata.placement)));
    }

    const std::optional<uint32_t> expected_stored_tile_size_opt = checked_mul<uint32_t>(metadata.halo_width, 2)
        .and_then([&](const uint32_t halo_widths) { return checked_add(metadata.nominal_tile_size, halo_widths); })
        .and_then([&](const uint32_t without_overlap) { return checked_add(without_overlap, metadata.overlap); });
    if (!is_power_of_two(metadata.nominal_tile_size) || metadata.halo_width > metadata.nominal_tile_size || expected_stored_tile_size_opt != metadata.stored_tile_size) {
        return alp::make_error(store::InvalidMetadata{}, fmt::format("the nominal size {}, stored size {}, halo width {} and overlap {} do not fit together", metadata.nominal_tile_size, metadata.stored_tile_size, metadata.halo_width, metadata.overlap));
    }

    if (!magic_enum::enum_contains(metadata.value_mapping)) {
        return alp::make_error(store::InvalidMetadata{}, fmt::format("the value mapping {} is unknown", std::to_underlying(metadata.value_mapping)));
    }

    return {};
}

alp::Result<Metadata, store::ReadMetadataError> read_metadata(const std::filesystem::path& base_path) {
    const std::filesystem::path path = base_path / metadata_file_name;
    const auto context = alp::make_context("reading the storage's metadata from {}", path);
    auto read_result = ::io::envelope::read<MetadataSchema>(path).transform_error(store::to_read_metadata_error);
    ALP_TRY_CONTEXT(v2::Metadata metadata, std::move(read_result), context);

    Metadata result{
        .ids = {
            .layout_id = std::move(metadata.layout_id),
            .payload_class = std::move(metadata.payload_class),
            .codec_id = std::move(metadata.codec_id),
        },
        .stored_tile_size = metadata.stored_tile_size,
        .nominal_tile_size = metadata.nominal_tile_size,
        .halo_width = metadata.halo_width,
        .value_mapping = metadata.value_mapping,
        .placement = metadata.placement,
        .overlap = metadata.overlap,
    };
    ALP_TRY_CONTEXT(alp::wrap_error(validate(result), store::CorruptMetadata{}), context);

    return result;
}

alp::Result<void, store::WriteMetadataError> write_metadata(const std::filesystem::path& base_path, const Metadata& metadata) {
    const std::filesystem::path path = base_path / metadata_file_name;
    const auto context = alp::make_context("writing the storage's metadata to {}", path);
    ALP_TRY_CONTEXT(alp::wrap_error(validate(metadata), store::UnencodableMetadata{}), context);

    const v2::Metadata encoded_metadata{
        .layout_id = metadata.ids.layout_id,
        .payload_class = metadata.ids.payload_class,
        .codec_id = metadata.ids.codec_id,
        .stored_tile_size = metadata.stored_tile_size,
        .nominal_tile_size = metadata.nominal_tile_size,
        .halo_width = metadata.halo_width,
        .value_mapping = metadata.value_mapping,
        .placement = metadata.placement,
        .overlap = metadata.overlap,
    };
    auto write_result = ::io::envelope::write<MetadataSchema>(encoded_metadata, path, {.mode = ::io::WriteMode::Overwrite}).transform_error(store::to_write_metadata_error);
    ALP_TRY_CONTEXT(std::move(write_result), context);
    return {};
}

} // namespace raster_store::storage
