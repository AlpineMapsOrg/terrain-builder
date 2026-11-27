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
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include <fmt/format.h>
#include <fmt/std.h>

#include "common_error_codes.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "store/Codec.h"
#include "store/Index.h"
#include "store/IndexFormat.h"
#include "store/Metadata.h"
#include "store/RawStorage.h"
#include "store/Storage.h"
#include "store/path_layout.h"
#include "store/storage_paths.h"

namespace store {

// Which states of a storage `open` accepts.
enum class Accept { Published, Draft, PublishedOrDraft };

using OpenError = alp::Error<InvalidStoragePath, ReadIndexError>;
// Opens the storage at `path`, whose node files `layout` locates and `codec` reads.
// Fails if the storage's state is not one `accept` allows.
template <Index IndexType, typename Payload>
[[nodiscard]]
alp::Result<Storage<IndexType, Payload>, OpenError> open(
    const std::filesystem::path& path,
    IndexFormat<IndexType> format,
    path_layout::Layout<typename IndexType::Traits::Key> layout,
    std::unique_ptr<Codec<Payload>> codec,
    const Accept accept = Accept::Published
) {
    using Key = typename IndexType::Traits::Key;

    const auto context = alp::make_context("opening the storage at {}", path);

    // Check the state
    const bool draft = is_draft(path);
    if (draft && accept == Accept::Published) {
        return alp::make_error(InvalidStoragePath{}, "the path names a draft").context(context);
    }
    if (!draft && accept == Accept::Draft) {
        return alp::make_error(InvalidStoragePath{}, "the path does not name a draft").context(context);
    }

    // Read the index
    ALP_TRY_CONTEXT(IndexType index, format.read(path), context);
    ALP_TRY_CONTEXT(validate_index(index, codec->supports_hashing()), context);

    // Create the storage
    RawStorage<typename IndexType::Traits, Payload> raw(path_layout::Resolver<Key>(path, layout), std::move(codec));
    return Storage<IndexType, Payload>(std::move(raw), std::move(index));
}

using ResolveError = alp::Error<UnexpectedPayloadClass, Unsupported>;
// Opens the storage at `path` that holds payloads of class `payload_class`, with the layout and codec `metadata` records.
template <Index IndexType, typename Payload>
[[nodiscard]]
alp::Result<Storage<IndexType, Payload>, ResolveError, OpenError> open(
    const std::filesystem::path& path,
    IndexFormat<IndexType> format,
    const Metadata& metadata,
    const std::string_view payload_class,
    CodecResolver<Payload> auto resolve_codec,
    const Accept accept = Accept::Published
) {
    const auto context = alp::make_context("opening the storage of {} at {}", payload_class, path);

    if (metadata.payload_class != payload_class) {
        return alp::make_error(UnexpectedPayloadClass{metadata.payload_class, std::string(payload_class)}).context(context);
    }
    const std::optional<path_layout::Layout<typename IndexType::Traits::Key>> layout_opt = format.resolve_layout(metadata.layout_id);
    if (!layout_opt.has_value()) {
        return alp::make_error(Unsupported{}, fmt::format("the layout {} is unknown", metadata.layout_id)).context(context);
    }
    ALP_TRY_CONTEXT(std::unique_ptr<Codec<Payload>> codec, resolve_codec(metadata.codec_id), context);
    auto open_result = open<IndexType, Payload>(path, std::move(format), *layout_opt, std::move(codec), accept);
    ALP_TRY_CONTEXT(auto storage, std::move(open_result), context);
    return storage;
}

} // namespace store
