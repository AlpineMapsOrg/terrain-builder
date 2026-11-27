/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2024 Martin Braunsperger
 * Copyright (C) 2024 Adam Celarek-Litofcenko
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
#include <stop_token>

#include "common_error_codes.h"
#include "error/Result.h"
#include "gis/Dataset.h"
#include "gis/srs/System.h"
#include "inputs.h"
#include "mesh/storage.h"
#include "octree/Id.h"
#include "TileProvider.h"

namespace sf {

// Builds the patch that `inputs` describes and saves it to `output_path`.
// Saves nothing if the dataset holds no terrain there.
[[nodiscard]]
alp::Result<void> build_and_save_patch(const inputs::Patch &inputs, const std::filesystem::path &output_path);

/// Coarser nodes touch the earth's centre, where coverage in the dataset's SRS is unavailable.
constexpr octree::Id::Level min_target_level = 2;

// Checks that build_all_patches supports `target_level`.
[[nodiscard]]
alp::Result<void, Unsupported> check_target_level(uint32_t target_level);

// Builds the mesh of every node at `target_level` that holds terrain into `output`, linking the nodes the checkpoint has instead of building them.
// Once a stop is requested through `stop`, it finishes the active nodes and fails with Stopped.
[[nodiscard]]
alp::Result<void> build_all_patches(
    const gis::Dataset &dataset,
    octree::Id::Level target_level,
    const TileProvider *tile_provider,
    const gis::srs::System &mesh_srs,
    mesh::storage::Draft &output,
    const mesh::storage::Checkpoint &checkpoint = {},
    const std::stop_token &stop = {});

// Builds every node at the inputs' target level that holds terrain into a new SF storage at `output_path`, reusing the nodes of the draft at `checkpoint_path`, which must have been produced from `inputs`.
// Once a stop is requested through `stop`, it finishes the active nodes and fails with Stopped, keeping the draft.
[[nodiscard]]
alp::Result<void> build_and_publish_all_patches(
    const inputs::Build &inputs,
    const std::filesystem::path &output_path,
    const std::optional<std::filesystem::path> &checkpoint_path = std::nullopt,
    const std::stop_token &stop = {});

}
