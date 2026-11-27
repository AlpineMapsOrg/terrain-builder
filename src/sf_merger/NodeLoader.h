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

#include <libassert/assert.hpp>

#include "NodeLoader.h"
#include "mesh/SimpleMesh.h"
#include "mesh/clip.h"
#include "mesh/texture_trim.h"
#include "octree/Id.h"
#include "octree/Space.h"
#include "hierarchy/NodeStatusOrMissing.h"
#include "mesh/storage.h"

class NodeLoader {
public:
    NodeLoader(const mesh::storage::Storage &storage, octree::Space space) : _storage(storage), _space(space) {}

    hierarchy::NodeStatusOrMissing get_status(const octree::Id &id) const noexcept {
        return this->_storage.index().status(id);
    }

    // Try to retrieve or reconstruct node mesh by ID
    // TODO: return const ref instead?
    std::optional<SimpleMesh> load_node(const octree::Id &id) const {
        // Try storage
        auto mesh_opt = this->_storage.load(id);
        if (mesh_opt) {
            return mesh_opt.value();
        }

        // Try ancestors
        for (auto parent = id.parent(); parent.has_value(); parent = parent->parent()) {
            const auto &parent_id = *parent;

            // Try storage
            auto parent_mesh_opt = this->_storage.load(parent_id);
            if (parent_mesh_opt) {
                auto parent_mesh = parent_mesh_opt.value();
                const auto bounds = this->_space.get_node_bounds(id);
                SimpleMesh clipped = mesh::clip_on_bounds(parent_mesh, bounds);
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
    const mesh::storage::Storage &_storage;
    const octree::Space _space;
};
