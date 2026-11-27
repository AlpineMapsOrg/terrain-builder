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

#include <optional>
#include <ranges>
#include <unordered_map>
#include <utility>

#include <libassert/assert.hpp>

#include "error/Result.h"
#include "range_utils.h"
#include "store/Hash.h"
#include "store/Index.h"
#include "store/KeyTraits.h"

namespace store {

// An index that records which nodes have a payload and each one's hash.
template <KeyTraits T>
class SimpleIndex {
public:
    using Traits = T;
    using Key = typename T::Key;

    // Whether the node has a payload.
    bool has(const Key& key) const {
        return this->_hashes.contains(key);
    }

    // The node's recorded hash, if there is one.
    std::optional<PayloadHashView> hash(const Key& key) const {
        const auto hash_opt = find_value_ref(this->_hashes, key);
        // A node without a hash is recorded with an empty one.
        if (!hash_opt.has_value() || hash_opt->get().empty()) {
            return std::nullopt;
        }
        return hash_opt->get();
    }

    // Records that the node has a payload with `hash`.
    // Returns false if it already had one.
    [[nodiscard]]
    bool add(const Key& key, std::optional<PayloadHash> hash) {
        assert_valid<T>(key);
        ASSERT(!hash.has_value() || hash->size() >= min_hash_size);

        const bool inserted = this->_hashes.try_emplace(key, std::move(hash).value_or(PayloadHash{})).second;
        return inserted;
    }

    // A view of the keys of the nodes with a payload.
    // It refers to this index and is valid while the index lives unchanged.
    auto keys() const {
        return this->_hashes | std::views::keys;
    }

private:
    std::unordered_map<Key, PayloadHash, typename T::Hasher> _hashes;
};

} // namespace store
