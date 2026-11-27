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

#include <functional>
#include <optional>
#include <utility>

#include <libassert/assert.hpp>

#include "NodeLoader.h"
#include "error/prelude.h"
#include "mesh/SimpleMesh.h"
#include "octree/Id.h"
#include "optional_utils.h"
#include "hierarchy/NodeStatusOrMissing.h"

namespace merge {

template <hierarchy::NodeStatusOrMissing Status>
class NodeData {
public:
    constexpr explicit NodeData(octree::Id id, const NodeLoader &loader) : _id(id), _loader(loader) {}

    static constexpr hierarchy::NodeStatusOrMissing status() {
        return Status;
    }

    octree::Id id() const {
        return this->_id;
    }

    // Status::Leaf or Status::Inner guarantuess a node is present
    [[nodiscard]]
    alp::Result<std::reference_wrapper<const SimpleMesh>, NodeLoader::LoadError> mesh() const
    requires(Status == hierarchy::NodeStatusOrMissing::Leaf || Status == hierarchy::NodeStatusOrMissing::Inner)
    {
        ALP_TRY(const auto mesh_opt, this->load_mesh());
        return *ASSERT_VAL(mesh_opt);
    }

    // Status::Missing and Status::Virtual do not exist on disk, but may be the child of a leaf or inner node
    [[nodiscard]]
    alp::Result<std::optional<std::reference_wrapper<const SimpleMesh>>, NodeLoader::LoadError> mesh() const
    requires(Status == hierarchy::NodeStatusOrMissing::Missing || Status == hierarchy::NodeStatusOrMissing::Virtual)
    {
        return this->load_mesh();
    }

private:
    alp::Result<std::optional<std::reference_wrapper<const SimpleMesh>>, NodeLoader::LoadError> load_mesh() const {
        if (this->_mesh.has_value()) {
            return alp::ref(std::as_const(this->_mesh));
        } else {
            ALP_TRY(this->_mesh, this->_loader.load_node(this->_id));
            return alp::ref(std::as_const(this->_mesh));
        }
    }

    octree::Id _id;
    const NodeLoader &_loader;
    mutable std::optional<SimpleMesh> _mesh;
};

} // namespace merge
