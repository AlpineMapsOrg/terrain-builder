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

#include <cstdint>
#include <filesystem>
#include <optional>
#include <stop_token>
#include <string_view>

#include "Range.h"
#include "build_config.h"
#include "texturing.h"
#include "error/Error.h"
#include "error/Result.h"
#include "mesh/storage.h"
#include "stop.h"
#include "storage.h"

namespace dag {

// When building a DAG level, this determines which input nodes are considered.
enum class IncludeMode {
    CurrentOnly,        // when building level L, only include input nodes at exactly level L
    CurrentAndCoarser,  // when building level L, include input nodes at level L and any coarser level L+X
};

struct BuildOptions {
    uint32_t clusters_per_partition;
    std::optional<float> target_ratio;
    std::optional<float> relative_target_error;
    TextureOptions texture_options = {};
    octree::Id root_node = octree::Id::root();
    IncludeMode include_mode = IncludeMode::CurrentOnly;
    bool parallelize = false;
};

// A requirement on a DAG build's input or output that is not met.
enum class BuildIoError {
    // The input index is not a valid SF index.
    InvalidIndex,
    // A built node or its debug mesh could not be written.
    UnwritableOutput,
    // The checkpoint cannot be linked into the output.
    IncompatibleCheckpoint,
};

inline std::string_view format_as(const BuildIoError error) {
    switch (error) {
    case BuildIoError::InvalidIndex:
        return "the input index is not a valid SF index";
    case BuildIoError::UnwritableOutput:
        return "a built node or its debug mesh could not be written";
    case BuildIoError::IncompatibleCheckpoint:
        return "the checkpoint cannot be linked into the output";
    default:
        return "an unknown DAG build failure";
    }
}

using BuildError = alp::Error<BuildIoError, Stopped>;

// Debug meshes go beside the output as <output>-debug, also while it is built in <output>.part.
std::filesystem::path make_debug_mesh_path(const std::filesystem::path& output_path);

[[nodiscard]]
alp::Result<void, BuildError> build_full(
    const mesh::storage::Storage& input_storage,
    dag::storage::Draft& output_storage,
    const BuildOptions& options,
    const dag::storage::Checkpoint& checkpoint = {},
    mesh::storage::Draft* debug_output = nullptr,
    const std::stop_token& stop = {});

// Builds the levels into output_storage, linking nodes that the checkpoint has instead of building them.
// With a debug output, it also saves a mesh of each built node there.
// Once a stop is requested through `stop`, it finishes the active nodes and fails with Stopped.
[[nodiscard]]
alp::Result<void, BuildError> build_levels(
    const mesh::storage::Storage& input_storage,
    dag::storage::Draft& output_storage,
    const BuildOptions& options,
    const AnyRange<uint32_t>& level_range,
    const dag::storage::Checkpoint& checkpoint = {},
    mesh::storage::Draft* debug_output = nullptr,
    const std::stop_token& stop = {});

} // namespace dag
