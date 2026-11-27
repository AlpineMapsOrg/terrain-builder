/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2021 Adam Celarek-Litofcenko
 * Copyright (C) 2021 Martin Braunsperger
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

#include <radix/geometry.h>
#include <radix/tile.h>

#include "error/Error.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "gis/Dataset.h"
#include "gis/srs/System.h"
#include "mesh/SimpleMesh.h"
#include "border.h"
#include "TileProvider.h"

namespace sf {

// The request lies outside the dataset.
struct OutOfBounds : alp::MessageCode<"the request lies outside the dataset"> {};

// The request lies inside the dataset, which holds no terrain there.
struct EmptyRegion : alp::MessageCode<"the requested region holds no terrain"> {};

// The dataset holds no terrain for the request.
using NoTerrain = alp::Error<OutOfBounds, EmptyRegion>;

// The dataset is not supported.
struct UnsupportedDataset : alp::MessageCode<"the dataset is not supported"> {};

// The heights in the requested region cannot be read.
struct UnreadableHeights : alp::MessageCode<"the heights in the requested region cannot be read"> {};

// The clip is not supported.
struct UnsupportedClip : alp::MessageCode<"the clip is not supported"> {};

// The mesh cannot be transformed.
struct UntransformableMesh : alp::MessageCode<"the mesh cannot be transformed"> {};

// An error from building a mesh from a height dataset.
using BuildMeshError = alp::Error<NoTerrain, UnsupportedDataset, UnreadableHeights, UnsupportedClip, UntransformableMesh>;

/// Builds the mesh of the dataset's terrain within the clip bounds, with its positions in `mesh_srs`.
/// Its UVs span `texture_bounds`, which is set to the bounds of the mesh in `texture_srs`.
[[nodiscard]]
alp::Result<SimpleMesh, BuildMeshError> build_reference_mesh_patch(
    gis::Dataset& dataset,
    const gis::srs::System& mesh_srs,
    const gis::srs::System& clip_srs, const radix::geometry::Aabb3d& clip_bounds,
    const gis::srs::System& texture_srs, radix::tile::SrsBounds& texture_bounds);

using BuildPatchError = alp::Error<BuildMeshError, TileError>;
// Builds the mesh of the dataset's terrain within the target bounds, textured from the tiles if there is a provider.
[[nodiscard]]
alp::Result<SimpleMesh, BuildPatchError> build_patch(
    gis::Dataset& dataset,
    const gis::srs::System& target_bounds_srs,
    const radix::geometry::Aabb3d& target_bounds,
    const TileProvider* tile_provider,
    const gis::srs::System& mesh_srs);
}
