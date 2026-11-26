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
#include <optional>
#include <ranges>
#include <utility>

#include <fmt/format.h>

#include "error/MessageCode.h"
#include "error/Result.h"
#include "error/try.h"
#include "store/Hash.h"
#include "store/KeyTraits.h"

namespace store {

struct CorruptIndex : alp::MessageCode<"the storage index is not valid"> {};

// An input range whose elements are `T`.
template <typename R, typename T>
concept InputRangeOf = std::ranges::input_range<R> && std::same_as<std::ranges::range_value_t<R>, T>;

// What a storage records about its nodes: which have a payload, and their hashes.
template <typename I>
concept Index = std::default_initializable<I> && std::copyable<I>
    && KeyTraits<typename I::Traits>
    && requires(I& index, const I& const_index, const typename I::Traits::Key& key, std::optional<PayloadHash> hash) {
    // Whether the node has a payload.
    { const_index.has(key) } -> std::same_as<bool>;
    // The node's recorded hash, if there is one.
    { const_index.hash(key) } -> std::same_as<std::optional<PayloadHashView>>;
    // Records that the node has a payload with `hash`.
    // Returns false if it already had one.
    { index.add(key, std::move(hash)) } -> std::same_as<bool>;
    // The keys of the nodes with a payload.
    { const_index.keys() } -> InputRangeOf<typename I::Traits::Key>;
};

// Checks that
// - every key is valid, and
// - every node with a payload has a hash
// - every present hash has at least min_hash_size bytes
// - hashes are present iff the codec supports it
template <Index I>
[[nodiscard]]
alp::Result<void, CorruptIndex> validate_index(const I& index, const bool codec_supports_hashing) {
    using Key = typename I::Traits::Key;

    // Validate keys
    for (const Key& key : index.keys()) {
        if (!I::Traits::is_valid(key)) {
            return alp::make_error(CorruptIndex{}, fmt::format("the index lists the invalid key {}", key));
        }
    }

    // Validate hashes
    for (const Key& key : index.keys()) {
        const std::optional<PayloadHashView> hash_opt = index.hash(key);

        if (!codec_supports_hashing && hash_opt.has_value()) {
            return alp::make_error(CorruptIndex{}, "the index holds hashes, but the storage's codec does not hash");
        }
        if (codec_supports_hashing && !hash_opt.has_value()) {
            return alp::make_error(CorruptIndex{}, fmt::format("node {} has a payload but no hash", key));
        }

        if (hash_opt.has_value() && hash_opt->size() < min_hash_size) {
            return alp::make_error(CorruptIndex{}, fmt::format("the hash of node {} has {} bytes, fewer than {}", key, hash_opt->size(), min_hash_size));
        }
    }

    return {};
}

} // namespace store
