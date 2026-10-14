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

#include <filesystem>
#include <string>
#include <utility>

#include <expected>

#include "octree/StoreTraits.h"
#include "octree/storage/IndexFile.h"
#include "store/IndexedStorage.h"
#include "store/open.h"
#include "store/snapshot.h"

namespace octree::storage {

// Creates an empty storage in <final_path>.part; see store::snapshot.
template <typename NodeData, typename CodecResolver>
Expected<store::IndexedStorage<StoreTraits, NodeData>> create(const std::filesystem::path& final_path,
    std::string payload_class,
    std::string codec_selector,
    CodecResolver resolve_codec,
    const store::path_layout::Mapping<Id> mapping)
{
    auto codec = resolve_codec(codec_selector);
    if (!codec) {
        return Error::propagate(std::move(codec), "resolve octree storage codec \"" + codec_selector + "\"");
    }
    auto partial = store::snapshot::create_partial(final_path);
    if (!partial) {
        return Error::propagate(std::move(partial));
    }
    auto storage
        = store::make_storage<StoreTraits, NodeData>(*partial, index_format(), mapping, std::move(payload_class), std::move(codec_selector), std::move(*codec));
    if (!storage) {
        return Error::propagate(std::move(storage));
    }
    // Even an empty incomplete snapshot has a readable checkpoint.
    if (auto saved = storage->save_index(); !saved) {
        return Error::propagate(std::move(saved), "create initial octree index");
    }
    return store::IndexedStorage<StoreTraits, NodeData>(std::move(*storage));
}

// Saves the index and renames <final>.part to <final>.
template <typename NodeData>
Expected<void> publish(store::IndexedStorage<StoreTraits, NodeData> storage)
{
    if (auto saved = storage.save_index(); !saved) {
        return Error::propagate(std::move(saved), "save octree index before publication");
    }
    return store::snapshot::publish(storage.base_path());
}

} // namespace octree::storage
