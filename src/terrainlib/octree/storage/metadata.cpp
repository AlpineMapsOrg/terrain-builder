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

#include "octree/storage/metadata.h"

#include <filesystem>
#include <string>
#include <string_view>
#include <utility>

#include <fmt/format.h>
#include <fmt/std.h>
#include <libassert/assert.hpp>

#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/WriteMode.h"
#include "io/compression.h"
#include "io/envelope.h"
#include "io/serde.h"
#include "store/Metadata.h"
#include "store/envelope_utils.h"

namespace octree::storage {

namespace {

namespace v1 {

    struct StoreMetadata {
        std::string layout_id;
        std::string payload_class;
        std::string codec_id;
    };

} // namespace v1

using StoreMetadataSchema = io::envelope::PayloadSchema<"octree.StoreMetadata", io::envelope::Version<1, v1::StoreMetadata>>;
using StoreMetadata = StoreMetadataSchema::Latest;

} // namespace

alp::Result<store::Metadata, store::ReadMetadataError> read_metadata(const std::filesystem::path& base_path) {
    const std::filesystem::path path = base_path / metadata_file_name;
    const auto context = alp::make_context("reading the storage's metadata from {}", path);
    auto read_result = io::envelope::read<StoreMetadataSchema>(path).transform_error(store::to_read_metadata_error);
    ALP_TRY_CONTEXT(StoreMetadata metadata, std::move(read_result), context);

    store::Metadata result{
        .layout_id = std::move(metadata.layout_id),
        .payload_class = std::move(metadata.payload_class),
        .codec_id = std::move(metadata.codec_id),
    };
    ALP_TRY_CONTEXT(alp::wrap_error(validate(result), store::CorruptMetadata{}), context);

    return result;
}

alp::Result<void, store::WriteMetadataError> write_metadata(const std::filesystem::path& base_path, const store::Metadata& metadata) {
    const std::filesystem::path path = base_path / metadata_file_name;
    const auto context = alp::make_context("writing the storage's metadata to {}", path);
    ALP_TRY_CONTEXT(alp::wrap_error(validate(metadata), store::UnencodableMetadata{}), context);

    const StoreMetadata encoded_metadata{
        .layout_id = metadata.layout_id,
        .payload_class = metadata.payload_class,
        .codec_id = metadata.codec_id,
    };
    auto write_result = io::envelope::write<StoreMetadataSchema>(encoded_metadata, path, {.mode = io::WriteMode::Replace}).transform_error(store::to_write_metadata_error);
    ALP_TRY_CONTEXT(std::move(write_result), context);
    return {};
}

} // namespace octree::storage
