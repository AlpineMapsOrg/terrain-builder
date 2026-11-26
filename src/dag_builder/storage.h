/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Martin Braunsperger
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

#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

#include <fmt/format.h>

#include "codec.h"
#include "common_error_codes.h"
#include "dag_node.h"
#include "error/Error.h"
#include "error/Result.h"
#include "error/try.h"
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

namespace dag::storage {

inline constexpr std::string_view payload_class = "dag.ClusterBatch";

namespace detail {

// The codec that `id` names, reading the clusters of a node.
[[nodiscard]]
inline alp::Result<std::unique_ptr<store::Codec<dag::ClusterBatch>>, Unsupported> resolve_cluster_batch_codec(const std::string_view id) {
    if (id == codec::id) {
        return std::make_unique<codec::ClusterBatch>();
    } else {
        return alp::make_error(Unsupported{}, fmt::format("the codec {} is unknown", id));
    }
}

// The codec that `id` names, reading only the metadata of a node.
[[nodiscard]]
inline alp::Result<std::unique_ptr<store::Codec<dag::NodeMetadata>>, Unsupported> resolve_metadata_codec(const std::string_view id) {
    if (id == codec::id) {
        return std::make_unique<codec::Metadata>();
    } else {
        return alp::make_error(Unsupported{}, fmt::format("the codec {} is unknown", id));
    }
}

} // namespace detail

using Storage = store::Storage<octree::storage::Index, dag::ClusterBatch>;
using Draft = store::Draft<octree::storage::Index, dag::ClusterBatch>;
using Checkpoint = store::Checkpoint<octree::storage::Index, dag::ClusterBatch>;
using MetadataStorage = store::Storage<octree::storage::Index, dag::NodeMetadata>;

using OpenError = octree::storage::OpenError;
// Opens the DAG storage at `path`, if its state is one `accept` allows.
[[nodiscard]]
inline alp::Result<Storage, OpenError> open(const std::filesystem::path& path, const store::Accept accept = store::Accept::Published) {
    ALP_TRY(Storage storage, octree::storage::open<dag::ClusterBatch>(path, payload_class, detail::resolve_cluster_batch_codec, accept));
    return storage;
}

// Opens the node metadata of the DAG storage at `path`, if its state is one `accept` allows.
[[nodiscard]]
inline alp::Result<MetadataStorage, OpenError> open_metadata(const std::filesystem::path& path, const store::Accept accept = store::Accept::Published) {
    auto open_result = octree::storage::open<dag::NodeMetadata>(path, payload_class, detail::resolve_metadata_codec, accept);
    ALP_TRY(MetadataStorage storage, std::move(open_result));
    return storage;
}

using CreateError = octree::storage::CreateError;
// Makes an empty DAG draft at `<final_path>.part`, whose node files `layout` places.
[[nodiscard]]
inline alp::Result<Draft, CreateError> create(const std::filesystem::path& final_path, const store::path_layout::Layout<octree::Id> layout) {
    ALP_TRY(Draft draft, octree::storage::create<dag::ClusterBatch>(final_path, std::string(payload_class), layout, std::make_unique<codec::ClusterBatch>()));
    return draft;
}

// Saves the draft's index, renames `<path>.part` to `<path>` and removes the input record.
[[nodiscard]]
inline alp::Result<void, store::PublishError> publish(Draft draft) {
    ALP_TRY(octree::storage::publish(std::move(draft)));
    return {};
}

} // namespace dag::storage
