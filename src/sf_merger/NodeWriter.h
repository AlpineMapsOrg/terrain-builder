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

#include <stop_token>
#include <utility>

#include <libassert/assert.hpp>

#include "NodeLoader.h"
#include "error/Context.h"
#include "error/Result.h"
#include "error/try.h"
#include "hierarchy/traverse.h"
#include "mesh/SimpleMesh.h"
#include "mesh/storage.h"
#include "mesh/validate/debug_assert.h"
#include "octree/Id.h"
#include "stop.h"
#include "store/Checkpointer.h"

class NodeWriter {
public:
    NodeWriter(mesh::storage::Draft &storage) : _storage(storage) {}

    [[nodiscard]]
    alp::Result<void, mesh::storage::Draft::SaveError> write_node(const octree::Id &id, const SimpleMesh &mesh) {
        mesh::debug_assert_valid(mesh);
        ALP_TRY(this->_storage.save(id, mesh));
        this->checkpoint_if_due();
        return {};
    }

    [[nodiscard]]
    alp::Result<void, mesh::storage::Draft::LinkError> link_node(const octree::Id &id, const mesh::storage::Storage &source) {
        ALP_TRY(this->_storage.link_from(id, source));
        this->checkpoint_if_due();
        return {};
    }

    [[nodiscard]]
    alp::Result<void, mesh::storage::Draft::LinkError> link_node(const octree::Id &id, const mesh::storage::Checkpoint &checkpoint) {
        ALP_TRY(this->_storage.link_from(id, checkpoint));
        this->checkpoint_if_due();
        return {};
    }

    using LinkSubtreeError = alp::Error<Stopped, mesh::storage::Draft::LinkError>;
    // Links the leaves of the subtree of `loader` rooted at `id` into the output, and stops at the first failure or once a stop is requested through `stop`.
    [[nodiscard]]
    alp::Result<void, LinkSubtreeError> link_subtree_to_output(const octree::Id &id, const NodeLoader &loader, const std::stop_token &stop) {
        auto traverse_result = hierarchy::traverse(
            loader.storage().index().map(),
            [&](const octree::Id &child_id, const hierarchy::NodeStatus status) -> alp::Result<void, LinkSubtreeError> {
                if (status == hierarchy::NodeStatus::Virtual) {
                    return {};
                }
                DEBUG_ASSERT(status == hierarchy::NodeStatus::Leaf);

                ALP_TRY(check_not_stopped(stop));
                ALP_TRY(this->link_node(child_id, loader.storage()));
                return {};
            },
            hierarchy::AlwaysRefine{},
            id);
        ALP_TRY_CONTEXT(std::move(traverse_result), alp::make_context("linking the subtree rooted at {} into the output", id));
        return {};
    }

private:
    // Saves the index once the checkpoint interval has passed.
    void checkpoint_if_due() {
        this->_checkpointer.save_if_due_or_warn(this->_storage);
    }

    mesh::storage::Draft &_storage;
    store::Checkpointer _checkpointer;
};
