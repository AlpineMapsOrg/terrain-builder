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
#include <utility>

#include "error/Error.h"
#include "error/Result.h"
#include "error/try.h"
#include "octree/storage/Index.h"
#include "octree/storage/index_format.h"
#include "octree/storage/metadata.h"
#include "store/Codec.h"
#include "store/Metadata.h"
#include "store/Storage.h"
#include "store/open.h"

namespace octree::storage {

using OpenError = alp::Error<store::ReadMetadataError, store::ResolveError, store::OpenError>;
// Opens the octree storage at `path` that holds payloads of class `payload_class`, with the layout and codec its metadata records.
template <typename Payload>
[[nodiscard]]
alp::Result<store::Storage<Index, Payload>, OpenError> open(
    const std::filesystem::path& path,
    const std::string_view payload_class,
    store::CodecResolver<Payload> auto resolve_codec,
    const store::Accept accept = store::Accept::Published
) {
    const auto context = alp::make_context("opening the octree storage of {} at {}", payload_class, path);
    ALP_TRY_CONTEXT(const store::Metadata metadata, read_metadata(path), context);
    auto open_result = store::open<Index, Payload>(path, index_format(), metadata, payload_class, resolve_codec, accept);
    ALP_TRY_CONTEXT(auto storage, std::move(open_result), context);
    return storage;
}

} // namespace octree::storage
