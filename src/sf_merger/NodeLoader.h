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

#include <optional>
#include <utility>

#include <libassert/assert.hpp>

#include "mesh/SimpleMesh.h"
#include "mesh/clip.h"
#include "mesh/texture_trim.h"
#include "octree/Id.h"
#include "octree/Space.h"
#include "hierarchy/NodeStatusOrMissing.h"
#include "error/prelude.h"
#include "mesh/storage.h"
#include "store/Codec.h"
#include "store/index_checks.h"

class NodeLoader {
public:
    NodeLoader(const mesh::storage::Storage &storage, octree::Space space) : _storage(storage), _space(space) {}

    hierarchy::NodeStatusOrMissing get_status(const octree::Id &id) const noexcept {
        return this->_storage.index().status(id);
    }

    using LoadError = store::CodecReadError;
    // Loads the mesh of the node, or clips the mesh of its nearest ancestor that has one.
    // Empty if neither the node nor any of its ancestors has a mesh.
    [[nodiscard]]
    alp::Result<std::optional<SimpleMesh>, LoadError> load_node(const octree::Id &id) const {
        // Try storage
        ALP_TRY(std::optional<SimpleMesh> mesh_opt, this->try_load(id));
        if (mesh_opt.has_value()) {
            return mesh_opt;
        }

        // Try ancestors
        for (auto parent = id.parent(); parent.has_value(); parent = parent->parent()) {
            const auto &parent_id = *parent;

            // Try storage
            ALP_TRY(const std::optional<SimpleMesh> parent_mesh_opt, this->try_load(parent_id));
            if (parent_mesh_opt.has_value()) {
                const auto bounds = this->_space.get_node_bounds(id);
                SimpleMesh clipped = mesh::clip_on_bounds(*parent_mesh_opt, bounds);
                trim_texture_inplace(clipped);
                return clipped;
            }
        }

        // Nothing found
        return std::nullopt;
    }

    const mesh::storage::Storage &storage() const {
        return this->_storage;
    }

private:
    // Loads the mesh of the node, or nothing if the storage does not have it.
    [[nodiscard]]
    alp::Result<std::optional<SimpleMesh>, LoadError> try_load(const octree::Id &id) const {
        auto load_result = this->_storage.load(id);
        if (load_result.has_value()) {
            return std::move(load_result).value();
        }

        // Only a node the index does not list is absent.
        ALP_TRY(std::move(load_result).error().split<store::NotFound>());
        return std::nullopt;
    }

    const mesh::storage::Storage &_storage;
    const octree::Space _space;
};
