/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2025 Martin Braunsperger
 * Copyright (C) 2025 Adam Celarek-Litofcenko
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

#include <libassert/assert.hpp>

#include "mesh/SimpleMesh.h"
#include "cli_support/log.h"
#include "hierarchy/NodeStatusOrMissing.h"
#include "mask.h"
#include "octree/Id.h"
#include "mesh/storage.h"
#include "octree/Space.h"
#include "utils.h"
#include "containers/Cow.h"
#include "sf/publish.h"
#include "sf/validate_index.h"

struct Context {
    const mesh::storage::Storage& input;
    mesh::storage::Draft& output;
    const octree::Space space;
    const bool keep_inside;
};

// A failure to read, link or save a node while cutting a subtree.
using CutNodeError = alp::Error<mesh::storage::Storage::LoadError, mesh::storage::Draft::LinkError, mesh::storage::Draft::SaveError>;

[[nodiscard]]
alp::Result<void, CutNodeError> cut_node(
    Context& ctx,
    const octree::Id& id,
    const MeshMask& mask);

[[nodiscard]]
inline alp::Result<void, CutNodeError> cut_leaf_node(
    Context& ctx,
    const octree::Id& id,
    const MeshMask& mask
) {
    DEBUG_ASSERT(ctx.input.index().status(id) == hierarchy::NodeStatus::Leaf);

    ALP_TRY(const SimpleMesh mesh, ctx.input.load(id));
    LOG_TRACE("Cutting mesh at {} using mask with {} vertices and {} triangles",
        id, mask.vertex_count(), mask.face_count());
    const Cow<const SimpleMesh> clipped = clip_on_mask(mesh, mask, ctx.keep_inside);
    if (clipped.is_ref()) {
        LOG_TRACE("Mesh was fully inside the mask");
        ALP_TRY(ctx.output.link_from(id, ctx.input));
    } else {
        const SimpleMesh &clipped_mesh = clipped;
        if (!clipped_mesh.is_empty()) {
            LOG_TRACE("Mesh was clipped from {} vertices and {} triangles to {} vertices and {} triangles",
                mesh.vertex_count(), mesh.face_count(), clipped_mesh.vertex_count(), clipped_mesh.face_count());
            ALP_TRY(ctx.output.save(id, clipped_mesh));
        } else {
            LOG_TRACE("Mesh was fully outside the mask");
        }
    }
    return {};
}

[[nodiscard]]
inline alp::Result<void, CutNodeError> cut_virtual_node(
    Context& ctx,
    const octree::Id& id,
    const MeshMask& mask
) {
    DEBUG_ASSERT(ctx.input.index().status(id) == hierarchy::NodeStatus::Virtual);
    const auto children = *ASSERT_VAL(id.children());
    for (const octree::Id& child_id : children) {
        // TODO: split mask based on inner planes instead
        const auto child_bounds = geometry::pad_bounds_relative(ctx.space.get_node_bounds(child_id), 0.125);
        LOG_TRACE("Clipping mask for {}", child_id);
        const MeshMask child_mask = clip_mask_on_bounds(mask, child_bounds);
        auto child_result = cut_node(ctx, child_id, child_mask);
        if (!child_result) {
            return child_result;
        }
    }
    return {};
}

[[nodiscard]]
inline alp::Result<void, CutNodeError> cut_node(
    Context& ctx,
    const octree::Id& id,
    const MeshMask& mask
) {
    if (ctx.keep_inside && mask.is_empty()) {
        LOG_TRACE("Mesh was fully outside the mask");
        return {};
    }

    /*
    Uncomment to output masks:
    const auto path = ctx.output.path_for(id);
    const auto new_path = path.parent_path() /
                          (path.stem().string() + "-mask" + path.extension().string());
    mesh::io::save_to_path(mask.mesh, new_path);
    */

    const hierarchy::NodeStatusOrMissing status = ctx.input.index().status(id);
    switch (status) {
    case hierarchy::NodeStatusOrMissing::Missing:
        return {};
    case hierarchy::NodeStatusOrMissing::Virtual:
        return cut_virtual_node(ctx, id, mask);
    case hierarchy::NodeStatusOrMissing::Leaf:
        return cut_leaf_node(ctx, id, mask);
    default:
        UNREACHABLE();
    }
}

using CutDatasetError = alp::Error<sf::UnexpectedInnerNode, CutNodeError>;
[[nodiscard]]
inline alp::Result<void, CutDatasetError> cut_dataset(
    const mesh::storage::Storage &input,
    const MeshMask& mask,
    mesh::storage::Draft &output,
    const bool keep_inside) {
    ALP_TRY_CONTEXT(sf::validate_index(input.index()), alp::make_context("checking the cut's input at {}", input.base_path()));
    Context ctx(input, output, octree::Space::earth(), keep_inside);
    ALP_TRY(cut_node(ctx, octree::Id::root(), mask));
    return {};
}

using CutError = alp::Error<mesh::storage::CreateError, CutDatasetError, sf::PublishError>;
// Cuts `input_dataset` with `mask` into a new SF storage at `output_path`.
[[nodiscard]]
inline alp::Result<void, CutError> cut_dataset(
    const mesh::storage::Storage &input_dataset,
    const MeshMask& mask,
    const std::filesystem::path &output_path,
    const bool keep_inside) {
    LOG_TRACE("Creating output dataset at {}", output_path);
    const auto context = alp::make_context("cutting the SF storage at {} into {}", input_dataset.base_path(), output_path);
    ALP_TRY_CONTEXT(mesh::storage::Draft output_dataset, mesh::storage::create(output_path, input_dataset.resolver().layout(), input_dataset.codec_id()), context);
    ALP_TRY_CONTEXT(cut_dataset(input_dataset, mask, output_dataset, keep_inside), context);
    ALP_TRY_CONTEXT(sf::publish(std::move(output_dataset)), context);
    return {};
}
