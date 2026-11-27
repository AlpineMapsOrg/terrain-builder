/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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

#include <string>

#include <fmt/format.h>

#include "error/Result.h"
#include "error/utils.h"
#include "hierarchy/NodeStatus.h"
#include "octree/Id.h"
#include "octree/storage/Index.h"

namespace sf {

// The index marks a node as inner, which the Structura Fundamentalis topology does not allow.
struct UnexpectedInnerNode {
    octree::Id node;

    bool operator==(const UnexpectedInnerNode&) const = default;
};

inline std::string format_as(const UnexpectedInnerNode& error) {
    return fmt::format("node {} is an inner node", error.node);
}

// Checks that the index holds only leaf and virtual nodes.
[[nodiscard]]
inline alp::Result<void, UnexpectedInnerNode> validate_index(const octree::storage::Index& index) {
    for (const auto& [key, status] : index.map()) {
        if (status == hierarchy::NodeStatus::Inner) {
            return alp::make_error(UnexpectedInnerNode{key});
        }
    }
    return {};
}

} // namespace sf
