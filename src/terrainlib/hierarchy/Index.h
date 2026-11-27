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

#include <cstddef>
#include <optional>
#include <ranges>
#include <unordered_map>
#include <utility>

#include <fmt/format.h>
#include <libassert/assert.hpp>
#include <magic_enum/magic_enum.hpp>

#include "error/Result.h"
#include "error/try.h"
#include "hierarchy/IndexMap.h"
#include "hierarchy/NodeStatus.h"
#include "range_utils.h"
#include "store/Hash.h"
#include "store/Index.h"

namespace hierarchy {

// A storage's index that also knows each node's status in the tree.
template <typename Keys, typename Tree>
class Index {
public:
    using Traits = Keys;
    using Key = typename Keys::Key;
    using HashMap = std::unordered_map<Key, store::PayloadHash, typename Keys::Hasher>;

    Index() = default;

    // The index with the statuses in `map` and the hashes in `hashes`, if they are consistent.
    [[nodiscard]]
    static alp::Result<Index, store::CorruptIndex> from_parts(IndexMap<Keys, Tree> map, HashMap hashes) {
        Index result(std::move(map), std::move(hashes));
        ALP_TRY(result.validate());
        return result;
    }

    // Whether the node has a payload.
    bool has(const Key& key) const {
        const std::optional<NodeStatus> status_opt = this->_map.get(key);
        return status_opt == NodeStatus::Leaf || status_opt == NodeStatus::Inner;
    }

    // The node's recorded hash, if there is one.
    std::optional<store::PayloadHashView> hash(const Key& key) const {
        const auto hash_opt = find_value_ref(this->_hashes, key);
        if (!hash_opt.has_value()) {
            return std::nullopt;
        }
        return hash_opt->get();
    }

    // The node's status in the tree, or nothing if the index does not list it.
    std::optional<NodeStatus> status(const Key& key) const {
        return this->_map.get(key);
    }

    // Records that the node has a payload with `hash`. Returns false if it already had one.
    [[nodiscard]]
    bool add(const Key& key, std::optional<store::PayloadHash> hash) {
        const bool added = this->_map.add(key);
        if (!added) {
            return false;
        }
        if (hash.has_value()) {
            ASSERT(hash->size() >= store::min_hash_size);
            this->_hashes.insert_or_assign(key, std::move(*hash));
        }
        return true;
    }

    // A view of the keys of the nodes with a payload. It refers to this index and is valid while the index lives unchanged.
    auto keys() const {
        return this->_map | std::views::filter([](const auto& e) { return e.second != NodeStatus::Virtual; }) | std::views::keys;
    }

    // The statuses of the index's nodes.
    const IndexMap<Keys, Tree>& map() const {
        return this->_map;
    }

private:
    Index(IndexMap<Keys, Tree> map, HashMap hashes)
        : _map(std::move(map)),
          _hashes(std::move(hashes)) {}

    // Checks that every status fits the node's children, every node except the root has an inner or virtual parent, and no node without a payload has a hash.
    [[nodiscard]]
    alp::Result<void, store::CorruptIndex> validate() const {
        for (const auto& [key, status] : this->_map) {
            if (!magic_enum::enum_contains<NodeStatus::Value>(status)) {
                return alp::make_error(store::CorruptIndex{}, fmt::format("node {} has the invalid status {}", key, std::to_underlying(static_cast<NodeStatus::Value>(status))));
            }
            bool has_child = false;
            if (const auto children = Tree::children(key)) {
                for (const Key& child : *children) {
                    // The children of a valid key are valid.
                    has_child = has_child || this->_map.get_raw(child) != nullptr;
                }
            }
            if (status == NodeStatus::Leaf && has_child) {
                return alp::make_error(store::CorruptIndex{}, fmt::format("node {} is a leaf with children", key));
            }
            if (status != NodeStatus::Leaf && !has_child) {
                return alp::make_error(store::CorruptIndex{}, fmt::format("node {} is {} without children", key, status));
            }
            const std::optional<Key> parent = Tree::parent(key);
            if (!parent) {
                if (key != Tree::root()) {
                    return alp::make_error(store::CorruptIndex{}, fmt::format("node {} has no parent but is not the root", key));
                }
                continue;
            }
            const NodeStatus* parent_status = this->_map.get_raw(*parent);
            if (parent_status == nullptr || *parent_status == NodeStatus::Leaf) {
                return alp::make_error(store::CorruptIndex{}, fmt::format("node {} has no inner or virtual parent in the index", key));
            }
        }
        for (const Key& key : this->_hashes | std::views::keys) {
            if (!this->has(key)) {
                return alp::make_error(store::CorruptIndex{}, fmt::format("node {} has a hash but no payload", key));
            }
        }
        return {};
    }

    IndexMap<Keys, Tree> _map;
    HashMap _hashes;
};

} // namespace hierarchy
