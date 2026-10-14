/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>

#include <expected>

#include "Error.h"
#include "store/Codec.h"
#include "store/Index.h"
#include "store/path_layout.h"

namespace store {

// Sparse content identities of physical nodes.
template <HierarchyTraits Traits>
using Key2Hash_Map = std::unordered_map<typename Traits::Key, NodeDataHash, typename Traits::Hasher>;

// The contents of an index payload: topology and the identities of physical nodes.
template <HierarchyTraits Traits>
struct IndexContents {
    Index<Traits> index;
    Key2Hash_Map<Traits> hashes;
};

template <HierarchyTraits Traits>
struct IndexMetadata {
    Index<Traits> index;
    Key2Hash_Map<Traits> hashes;
    std::string layout_id;
    std::string payload_class;
    std::string codec_selector;
};

// With a codec that hashes, every physical node has a hash of at least min_hash_size bytes
// and no other node has one. With a codec that does not hash, there are no hashes.
template <HierarchyTraits Traits>
Expected<void> validate_hashes(const Index<Traits>& index, const Key2Hash_Map<Traits>& hashes, const bool codec_hashes)
{
    if (!codec_hashes) {
        if (!hashes.empty()) {
            return Error::fail(Error::Code::CorruptData, "index contains hashes, but its codec does not support hashing");
        }
        return {};
    }
    std::size_t physical_count = 0;
    for (const auto& [key, status] : index) {
        if (status == NodeStatus::Virtual) {
            continue;
        }
        ++physical_count;
        const auto found = hashes.find(key);
        if (found == hashes.end()) {
            return Error::fail(Error::Code::CorruptData, "index lacks the hash of physical node " + Traits::key_to_string(key));
        }
        if (found->second.size() < min_hash_size) {
            return Error::fail(Error::Code::CorruptData, "index contains a hash of fewer than four bytes for node " + Traits::key_to_string(key));
        }
    }
    if (hashes.size() != physical_count) {
        return Error::fail(Error::Code::CorruptData, "index contains hashes of nodes that are not physically present");
    }
    return {};
}

template <HierarchyTraits Traits>
struct IndexFormat {
    std::string_view index_filename;

    Expected<IndexMetadata<Traits>> (*read)(const std::filesystem::path& index_path);
    Expected<void> (*write)(const std::filesystem::path& index_path, const IndexMetadata<Traits>& metadata);
    std::optional<path_layout::Mapping<typename Traits::Key>> (*mapping_from_id)(std::string_view id);
    path_layout::Mapping<typename Traits::Key> (*default_mapping)();
};

} // namespace store
