/*****************************************************************************
 * AlpineMaps.org
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

#include <filesystem>
#include <memory>
#include <utility>

#include <fmt/std.h>

#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/FileError.h"
#include "io/filesystem.h"
#include "path_utils.h"
#include "store/Codec.h"
#include "store/Index.h"
#include "store/IndexFormat.h"
#include "store/Draft.h"
#include "store/PersistedIndex.h"
#include "store/RawStorage.h"
#include "store/path_layout.h"
#include "store/storage_paths.h"

namespace store {

// Makes an empty draft at `<final_path>.part`, with an empty index.
// Fails if `<final_path>` or `<final_path>.part` exists.
template <Index IndexType, typename Payload>
[[nodiscard]]
alp::Result<Draft<IndexType, Payload>, CreateError> create(
    const std::filesystem::path& final_path,
    IndexFormat<IndexType> format,
    path_layout::Layout<typename IndexType::Traits::Key> layout,
    std::unique_ptr<Codec<Payload>> codec
) {
    using Key = typename IndexType::Traits::Key;

    const auto context = alp::make_context("creating the storage at {}", final_path);

    // Check paths
    const std::filesystem::path destination = paths::normalized(final_path);
    const std::filesystem::path name = destination.filename();
    if (name.empty() || name == "." || name == "..") {
        return alp::make_error(InvalidStoragePath{}, "the path does not end in a file name").context(context);
    }
    if (is_draft(destination)) {
        return alp::make_error(InvalidStoragePath{}, "the path names a draft").context(context);
    }

    // Check directories
    ALP_TRY_CONTEXT(require_absent(destination), context);
    const std::filesystem::path draft_path = make_draft_path(destination);
    ALP_TRY_CONTEXT(require_absent(draft_path), context);

    // Create dirs
    ALP_TRY_CONTEXT(io::filesystem::create_parent_directories(draft_path), context);
    // Another run may have made the directory since the check above; creating it fails then instead of sharing it.
    ALP_TRY_CONTEXT(const bool created, io::filesystem::create_directory(draft_path), context);
    if (!created) {
        return alp::make_error(io::FileError(std::errc::file_exists, draft_path)).context(context);
    }

    // Create storage
    RawStorage<typename IndexType::Traits, Payload> raw(path_layout::Resolver<Key>(draft_path, layout), std::move(codec));
    PersistedIndex<IndexType> index(IndexType{}, format, draft_path);
    Draft<IndexType, Payload> draft(std::move(raw), std::move(index));
    ALP_TRY_CONTEXT(draft.save_index(), context);

    return draft;
}

} // namespace store
