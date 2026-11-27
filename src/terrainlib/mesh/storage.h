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
#include <string>
#include <string_view>
#include <utility>

#include <fmt/std.h>

#include "common_error_codes.h"
#include "error/Error.h"
#include "error/Result.h"
#include "error/try.h"
#include "mesh/codec/resolve.h"
#include "mesh/SimpleMesh.h"
#include "octree/Id.h"
#include "octree/storage/Index.h"
#include "octree/storage/create.h"
#include "octree/storage/open.h"
#include "octree/storage/publish.h"
#include "store/Checkpoint.h"
#include "store/Draft.h"
#include "store/Storage.h"
#include "store/open.h"
#include "store/path_layout.h"
#include "store/publish.h"

namespace mesh::storage {

inline constexpr std::string_view payload_class = "mesh.Simple3d";

using Storage = store::Storage<octree::storage::Index, mesh::Simple>;
using Draft = store::Draft<octree::storage::Index, mesh::Simple>;
using Checkpoint = store::Checkpoint<octree::storage::Index, mesh::Simple>;

using OpenError = octree::storage::OpenError;
// Opens the mesh storage at `path`, if its state is one `accept` allows.
[[nodiscard]]
inline alp::Result<Storage, OpenError> open(const std::filesystem::path& path, const store::Accept accept = store::Accept::Published) {
    ALP_TRY(Storage storage, octree::storage::open<mesh::Simple>(path, payload_class, codec::resolve, accept));
    return storage;
}

using CreateError = alp::Error<Unsupported, octree::storage::CreateError>;
// Makes an empty mesh draft at `<final_path>.part`, whose node files `layout` places and the codec `codec_id` names encodes.
[[nodiscard]]
inline alp::Result<Draft, CreateError> create(const std::filesystem::path& final_path, const store::path_layout::Layout<octree::Id> layout, const std::string_view codec_id) {
    ALP_TRY_CONTEXT(auto codec, codec::resolve(codec_id), alp::make_context("choosing the codec for the mesh storage at {}", final_path));
    ALP_TRY(Draft draft, octree::storage::create<mesh::Simple>(final_path, std::string(payload_class), layout, std::move(codec)));
    return draft;
}

// Saves the draft's index, renames `<path>.part` to `<path>` and removes the input record.
[[nodiscard]]
inline alp::Result<void, store::PublishError> publish(Draft draft) {
    ALP_TRY(octree::storage::publish(std::move(draft)));
    return {};
}

} // namespace mesh::storage
