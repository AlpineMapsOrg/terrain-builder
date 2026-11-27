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

#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/FileError.h"
#include "store/Index.h"
#include "store/Draft.h"

namespace store {

namespace detail {

// Renames the draft at `draft_path` to its final path and removes its input record.
[[nodiscard]]
alp::Result<void, io::FileError> publish_directory(const std::filesystem::path& draft_path);

} // namespace detail

// Saves the draft's index, renames `<path>.part` to `<path>` and removes the input record.
template <Index IndexType, typename Payload>
[[nodiscard]]
alp::Result<void, PublishError> publish(Draft<IndexType, Payload> draft) {
    const auto context = alp::make_context("publishing the storage at {}", draft.base_path());
    ALP_TRY_CONTEXT(draft.save_index(), context);
    ALP_TRY_CONTEXT(detail::publish_directory(draft.base_path()), context);
    return {};
}

} // namespace store
