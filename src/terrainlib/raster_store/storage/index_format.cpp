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

#include "raster_store/storage/index_format.h"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <ranges>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <fmt/std.h>
#include <magic_enum/magic_enum.hpp>
#include <radix/tile.h>

#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "hierarchy/IndexMap.h"
#include "hierarchy/NodeStatus.h"
#include "io/WriteMode.h"
#include "io/envelope.h"
#include "range_utils.h"
#include "raster_store/StoreTraits.h"
#include "raster_store/path_layout.h"
#include "store/Hash.h"
#include "store/IndexFormat.h"
#include "store/envelope_utils.h"

namespace raster_store::storage {

namespace {

namespace v1 {

static_assert(sizeof(unsigned) == sizeof(uint32_t), "raster index v1 requires 32-bit tile ID components");

struct IndexEntry {
    radix::tile::Id id;
    uint8_t status;
};

struct HashEntry {
    radix::tile::Id id;
    store::PayloadHash hash;
};

struct Hierarchy {
    std::vector<IndexEntry> entries;
    std::vector<HashEntry> hashes;
};

} // namespace v1

using IndexSchema = ::io::envelope::PayloadSchema<"raster_store.Index", ::io::envelope::Version<1, v1::Hierarchy>>;
using Hierarchy = IndexSchema::Latest;

Hierarchy encode_index(const Index& index) {
    // Both lists are written in id order, so the tiles are sorted once.
    using Entry = std::pair<radix::tile::Id, hierarchy::NodeStatus>;
    std::vector<Entry> entries = to_vector<Entry>(index.map());
    std::ranges::sort(entries, std::less{}, &Entry::first);

    Hierarchy result;

    // The status of every tile, virtual ones included.
    result.entries.reserve(entries.size());
    for (const auto& [id, status] : entries) {
        result.entries.push_back({
            .id = id,
            .status = std::to_underlying(static_cast<hierarchy::NodeStatus::Value>(status)),
        });
    }

    // The hash of every tile that has one.
    for (const radix::tile::Id& id : entries | std::views::keys) {
        const std::optional<store::PayloadHashView> hash_opt = index.hash(id);
        if (!hash_opt.has_value()) {
            continue;
        }
        result.hashes.push_back({
            .id = id,
            .hash = std::ranges::to<store::PayloadHash>(*hash_opt),
        });
    }

    return result;
}

[[nodiscard]]
alp::Result<Index, store::CorruptIndex> decode_index(const Hierarchy& encoded) {
    hierarchy::IndexMap<KeyTraits, HierarchyTraits> map;
    for (const v1::IndexEntry& entry : encoded.entries) {
        if (!KeyTraits::is_valid(entry.id)) {
            return alp::make_error(store::CorruptIndex{}, fmt::format("the index holds the invalid tile {}", entry.id));
        }
        if (map.is_present(entry.id)) {
            return alp::make_error(store::CorruptIndex{}, fmt::format("the index holds {} twice", entry.id));
        }
        const std::optional<hierarchy::NodeStatus::Value> status_opt = magic_enum::enum_cast<hierarchy::NodeStatus::Value>(entry.status);
        if (!status_opt.has_value()) {
            return alp::make_error(store::CorruptIndex{}, fmt::format("tile {} has the invalid status {}", entry.id, entry.status));
        }
        map.set_raw(entry.id, *status_opt);
    }

    Index::HashMap hashes;
    for (const v1::HashEntry& entry : encoded.hashes) {
        if (!KeyTraits::is_valid(entry.id)) {
            return alp::make_error(store::CorruptIndex{}, fmt::format("the index holds a hash for the invalid tile {}", entry.id));
        }
        if (!hashes.emplace(entry.id, entry.hash).second) {
            return alp::make_error(store::CorruptIndex{}, fmt::format("the index holds two hashes for {}", entry.id));
        }
    }

    ALP_TRY(Index result, Index::from_parts(std::move(map), std::move(hashes)));
    return result;
}

} // namespace

alp::Result<Index, store::ReadIndexError> read_index(const std::filesystem::path& base_path) {
    const std::filesystem::path path = base_path / index_file_name;
    const auto context = alp::make_context("reading the storage's index from {}", path);
    auto read_result = ::io::envelope::read<IndexSchema>(path).transform_error(store::to_read_index_error);
    ALP_TRY_CONTEXT(const Hierarchy encoded_index, std::move(read_result), context);
    ALP_TRY_CONTEXT(Index index, decode_index(encoded_index), context);
    return index;
}

alp::Result<void, store::WriteIndexError> write_index(const std::filesystem::path& base_path, const Index& index) {
    const std::filesystem::path path = base_path / index_file_name;
    const auto context = alp::make_context("writing the storage's index to {}", path);
    const Hierarchy encoded_index = encode_index(index);
    auto write_result = ::io::envelope::write<IndexSchema>(encoded_index, path, {.mode = ::io::WriteMode::Replace}).transform_error(store::to_write_index_error);
    ALP_TRY_CONTEXT(std::move(write_result), context);
    return {};
}

store::IndexFormat<Index> index_format() {
    return {
        read_index,
        write_index,
        path_layout::resolve_layout,
        path_layout::zoom_x_y_google::layout,
    };
}

} // namespace raster_store::storage
