/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2025 Martin Braunsperger
 * Copyright (C) 2025 Adam Celarek-Litofcenko
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

#include <expected>
#include <functional>
#include <optional>

#include <libassert/assert.hpp>

#include "Error.h"
#include "NodeLoader.h"
#include "mesh/SimpleMesh.h"
#include "octree/Id.h"
#include "mesh/storage.h"
#include "store/traverse.h"

// TODO: make thread safe
class NodeWriter {
public:
    NodeWriter(mesh::storage::Storage &storage) : _storage(storage) {}

    Expected<void> write_node(
        const octree::Id &id,
        const SimpleMesh &mesh) {
        mesh::validate(mesh);
        return this->_storage.save(id, mesh);
    }

    Expected<void> link_node(const octree::Id& id, const mesh::storage::IndexedStorage& source) { return this->_storage.link_from(id, source); }

    Expected<void> save_index() { return this->_storage.save_index(); }

    // Calls poll before linking each leaf and stops at its first error, e.g. a cancellation.
    Expected<void> link_subtree_to_output(
        const octree::Id &id,
        const NodeLoader &loader,
        const std::function<Expected<void>()> &poll) {
        std::optional<Error> error = std::nullopt;
        auto traversal = store::traverse(
            loader.storage().index(),
            [&](const octree::Id &child_id, const store::NodeStatus &status) {
                if (error.has_value()) {
                    return;
                }
                if (status == store::NodeStatus::Virtual) {
                    return;
                }
                DEBUG_ASSERT(status == store::NodeStatus::Leaf);

                if (auto polled = poll(); !polled) {
                    error = std::move(polled).error();
                    return;
                }
                auto result = this->_storage.link_from(child_id, loader.storage());
                if (!result) {
                    error = std::move(result).error();
                }
            },
            [&](const octree::Id &) { return !error.has_value(); },
            id);
        if (!traversal) {
            return Error::propagate(
                std::move(traversal), "traverse source subtree rooted at " + id.to_string() + " while linking");
        }
        if (error.has_value()) {
            return Error::propagate(
                std::move(error.value()), "link subtree rooted at " + id.to_string() + " to output");
        }
        return {};
    }

private:
    mesh::storage::Storage &_storage;
};
