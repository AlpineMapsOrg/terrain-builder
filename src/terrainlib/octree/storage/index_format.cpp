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

#include "octree/storage/index_format.h"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <ranges>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <magic_enum/magic_enum.hpp>

#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "hierarchy/IndexMap.h"
#include "hierarchy/NodeStatus.h"
#include "io/WriteMode.h"
#include "io/envelope.h"
#include "octree/StoreTraits.h"
#include "octree/path_layout.h"
#include "range_utils.h"
#include "store/Hash.h"
#include "store/Index.h"
#include "store/IndexFormat.h"
#include "store/envelope_utils.h"

namespace octree::storage {

namespace {

namespace v1 {

struct IndexEntry {
    uint8_t level;
    uint64_t index;
    uint8_t status;
};

struct HashEntry {
    uint8_t level;
    uint64_t index;
    store::PayloadHash hash;
};

struct StoreIndex {
    std::vector<IndexEntry> entries;
    std::vector<HashEntry> hashes;
};

} // namespace v1

using StoreIndexSchema = io::envelope::PayloadSchema<"octree.StoreIndex", io::envelope::Version<1, v1::StoreIndex>>;
using StoreIndex = StoreIndexSchema::Latest;

StoreIndex encode_index(const Index& index) {
    // Both lists are written in id order, so the nodes are sorted once.
    using Entry = std::pair<Id, hierarchy::NodeStatus>;
    std::vector<Entry> entries = to_vector<Entry>(index.map());
    std::ranges::sort(entries, {}, &Entry::first);

    StoreIndex result;

    // The status of every node, virtual ones included.
    result.entries.reserve(entries.size());
    for (const auto& [id, status] : entries) {
        result.entries.push_back({
            .level = id.level(),
            .index = id.index_on_level(),
            .status = std::to_underlying(static_cast<hierarchy::NodeStatus::Value>(status)),
        });
    }

    // The hash of every node that has one.
    for (const Id& id : entries | std::views::keys) {
        const std::optional<store::PayloadHashView> hash_opt = index.hash(id);
        if (!hash_opt.has_value()) {
            continue;
        }
        result.hashes.push_back({
            .level = id.level(),
            .index = id.index_on_level(),
            .hash = std::ranges::to<store::PayloadHash>(*hash_opt),
        });
    }

    return result;
}

[[nodiscard]]
alp::Result<Index, store::CorruptIndex> decode_index(const StoreIndex& encoded) {
    hierarchy::IndexMap<KeyTraits, HierarchyTraits> map;
    for (const v1::IndexEntry& entry : encoded.entries) {
        const auto id_opt = Id::try_make(static_cast<Id::Level>(entry.level), static_cast<Id::Index>(entry.index));
        if (!id_opt) {
            return alp::make_error(store::CorruptIndex{}, fmt::format("the index holds the invalid id {} on level {}", entry.index, entry.level));
        }
        if (map.is_present(*id_opt)) {
            return alp::make_error(store::CorruptIndex{}, fmt::format("the index holds {} twice", *id_opt));
        }
        const std::optional<hierarchy::NodeStatus::Value> status_opt = magic_enum::enum_cast<hierarchy::NodeStatus::Value>(entry.status);
        if (!status_opt.has_value()) {
            return alp::make_error(store::CorruptIndex{}, fmt::format("node {} has the invalid status {}", *id_opt, entry.status));
        }
        map.set_raw(*id_opt, *status_opt);
    }

    Index::HashMap hashes;
    for (const v1::HashEntry& entry : encoded.hashes) {
        const auto id_opt = Id::try_make(static_cast<Id::Level>(entry.level), static_cast<Id::Index>(entry.index));
        if (!id_opt) {
            return alp::make_error(store::CorruptIndex{}, fmt::format("the index holds a hash for the invalid id {} on level {}", entry.index, entry.level));
        }
        if (!hashes.emplace(*id_opt, entry.hash).second) {
            return alp::make_error(store::CorruptIndex{}, fmt::format("the index holds two hashes for {}", *id_opt));
        }
    }

    ALP_TRY(Index result, Index::from_parts(std::move(map), std::move(hashes)));
    return result;
}

} // namespace

alp::Result<Index, store::ReadIndexError> read_index(const std::filesystem::path& base_path) {
    const std::filesystem::path path = base_path / index_file_name;
    const auto context = alp::make_context("reading the storage's index from {}", path);
    auto read_result = io::envelope::read<StoreIndexSchema>(path).transform_error(store::to_read_index_error);
    ALP_TRY_CONTEXT(const StoreIndex encoded_index, std::move(read_result), context);
    ALP_TRY_CONTEXT(Index index, decode_index(encoded_index), context);
    return index;
}

alp::Result<void, store::WriteIndexError> write_index(const std::filesystem::path& base_path, const Index& index) {
    const std::filesystem::path path = base_path / index_file_name;
    const auto context = alp::make_context("writing the storage's index to {}", path);
    const StoreIndex encoded_index = encode_index(index);
    auto write_result = io::envelope::write<StoreIndexSchema>(encoded_index, path, {.mode = io::WriteMode::Replace}).transform_error(store::to_write_index_error);
    ALP_TRY_CONTEXT(std::move(write_result), context);
    return {};
}

store::path_layout::Layout<Id> default_layout() {
    return path_layout::level_and_coordinate_directories::layout();
}

store::IndexFormat<Index> index_format() {
    return {
        read_index,
        write_index,
        path_layout::resolve_layout,
        default_layout,
    };
}

} // namespace octree::storage
