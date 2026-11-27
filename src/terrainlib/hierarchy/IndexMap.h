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

#include <concepts>
#include <cstddef>
#include <optional>
#include <unordered_map>
#include <utility>

#include <fmt/format.h>
#include <libassert/assert.hpp>

#include "error/Result.h"
#include "error/utils.h"
#include "hierarchy/NodeStatus.h"
#include "hierarchy/Traits.h"
#include "store/Index.h"
#include "store/KeyTraits.h"

namespace hierarchy {

// The status of every node in a tree of keys, kept consistent as nodes gain and lose payloads.
// Every function taking a key requires it to be valid (Keys::is_valid).
template <store::KeyTraits Keys, Traits Tree>
    requires std::same_as<typename Keys::Key, typename Tree::Key>
class IndexMap {
public:
    using Key = typename Keys::Key;
    using Container = std::unordered_map<Key, NodeStatus, typename Keys::Hasher>;
    using iterator = typename Container::iterator;
    using const_iterator = typename Container::const_iterator;

    std::optional<NodeStatus> get(const Key& key) const {
        store::assert_valid<Keys>(key);
        const NodeStatus* status = get_raw_unchecked(key);
        if (status == nullptr) {
            return std::nullopt;
        }
        return *status;
    }

    bool add(const Key& key) {
        store::assert_valid<Keys>(key);

        NodeStatus* status = get_raw_unchecked(key);
        if (status != nullptr) {
            switch (*status) {
            case NodeStatus::Inner:
            case NodeStatus::Leaf:
                return false;
            case NodeStatus::Virtual:
                set_raw_unchecked(key, NodeStatus::Inner);
                return true;
            }
        }

        const auto parent = Tree::parent(key);
        if (!parent.has_value()) {
            DEBUG_ASSERT(key == Tree::root());
            if (empty()) {
                set_raw_unchecked(key, NodeStatus::Leaf);
                return true;
            }
            UNREACHABLE();
        }

        NodeStatus* parent_status = get_raw_unchecked(parent.value());
        if (parent_status == nullptr) {
            add(parent.value());
            set_raw_unchecked(parent.value(), NodeStatus::Virtual);
            set_raw_unchecked(key, NodeStatus::Leaf);
            return true;
        }

        switch (*parent_status) {
        case NodeStatus::Leaf:
            set_raw_unchecked(parent.value(), NodeStatus::Inner);
            set_raw_unchecked(key, NodeStatus::Leaf);
            break;
        case NodeStatus::Inner:
        case NodeStatus::Virtual:
            set_raw_unchecked(key, NodeStatus::Leaf);
            break;
        }
        return true;
    }

    bool remove(const Key& key) {
        store::assert_valid<Keys>(key);

        NodeStatus* status = get_raw_unchecked(key);
        if (status == nullptr) {
            return false;
        }
        switch (*status) {
        case NodeStatus::Inner:
            *status = NodeStatus::Virtual;
            return true;
        case NodeStatus::Virtual:
            return false;
        case NodeStatus::Leaf:
            remove_raw_unchecked(key);
            return update_parent_after_remove(key);
        }
        UNREACHABLE();
    }

    bool is_present(const Key& key) const {
        store::assert_valid<Keys>(key);
        return m_index.contains(key);
    }
    bool is_absent(const Key& key) const { return !is_present(key); }
    bool is(const NodeStatus status, const Key& key) const { return get(key) == status; }

    NodeStatus* get_raw(const Key& key) {
        store::assert_valid<Keys>(key);
        return get_raw_unchecked(key);
    }
    const NodeStatus* get_raw(const Key& key) const {
        store::assert_valid<Keys>(key);
        return get_raw_unchecked(key);
    }
    void set_raw(const Key& key, const NodeStatus status) {
        store::assert_valid<Keys>(key);
        set_raw_unchecked(key, status);
    }
    void remove_raw(const Key& key) {
        store::assert_valid<Keys>(key);
        remove_raw_unchecked(key);
    }

    void clear() { m_index.clear(); }
    bool empty() const { return m_index.empty(); }
    size_t size() const { return m_index.size(); }

    const_iterator begin() const { return m_index.begin(); }
    const_iterator end() const { return m_index.end(); }
    const_iterator cbegin() const { return m_index.cbegin(); }
    const_iterator cend() const { return m_index.cend(); }

private:
    NodeStatus* get_raw_unchecked(const Key& key) {
        const auto iterator = m_index.find(key);
        return iterator == m_index.end() ? nullptr : &iterator->second;
    }
    const NodeStatus* get_raw_unchecked(const Key& key) const {
        const auto iterator = m_index.find(key);
        return iterator == m_index.end() ? nullptr : &iterator->second;
    }
    void set_raw_unchecked(const Key& key, const NodeStatus status) { m_index[key] = status; }
    void remove_raw_unchecked(const Key& key) { m_index.erase(key); }

    bool update_parent_after_remove(const Key& key) {
        const auto parent = Tree::parent(key);
        if (!parent.has_value()) {
            return true;
        }

        NodeStatus* parent_status = get_raw_unchecked(parent.value());
        DEBUG_ASSERT(parent_status != nullptr);
        const auto siblings = Tree::children(parent.value());
        DEBUG_ASSERT(siblings.has_value());
        for (const Key& sibling : siblings.value()) {
            if (sibling != key && get_raw_unchecked(sibling) != nullptr) {
                return true;
            }
        }

        switch (*parent_status) {
        case NodeStatus::Inner:
            *parent_status = NodeStatus::Leaf;
            break;
        case NodeStatus::Virtual:
            remove_raw_unchecked(parent.value());
            return update_parent_after_remove(parent.value());
        case NodeStatus::Leaf:
            UNREACHABLE();
        }
        return true;
    }

    Container m_index;
};

} // namespace hierarchy
