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
#include <memory>
#include <string>
#include <utility>

#include <libassert/assert.hpp>

#include "error/Error.h"
#include "error/Result.h"
#include "error/try.h"
#include "octree/Id.h"
#include "octree/storage/Index.h"
#include "octree/storage/index_format.h"
#include "octree/storage/metadata.h"
#include "store/Codec.h"
#include "store/Metadata.h"
#include "store/Draft.h"
#include "store/create.h"
#include "store/path_layout.h"

namespace octree::storage {

using CreateError = alp::Error<store::CreateError, store::WriteMetadataError>;
// Makes an empty octree draft at `<final_path>.part` that records its layout, payload class and codec.
template <typename Payload>
[[nodiscard]]
alp::Result<store::Draft<Index, Payload>, CreateError> create(
    const std::filesystem::path& final_path,
    std::string payload_class,
    store::path_layout::Layout<Id> layout,
    std::unique_ptr<store::Codec<Payload>> codec
) {
    const auto context = alp::make_context("creating the octree storage of {} at {}", payload_class, final_path);

    const store::Metadata metadata{
        .layout_id = std::string(layout.id),
        .payload_class = payload_class,
        .codec_id = std::string(codec->id()),
    };
    DEBUG_ASSERT(validate(metadata).has_value());

    auto create_result = store::create<Index, Payload>(final_path, index_format(), layout, std::move(codec));
    ALP_TRY_CONTEXT(auto draft, std::move(create_result), context);
    ALP_TRY_CONTEXT(write_metadata(draft.base_path(), metadata), context);

    return draft;
}

} // namespace octree::storage
