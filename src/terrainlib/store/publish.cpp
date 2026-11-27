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

#include "store/publish.h"

#include <libassert/assert.hpp>

#include "cli_support/log.h"
#include "error/format.h"
#include "io/filesystem.h"
#include "path_utils.h"
#include "store/inputs.h"
#include "store/storage_paths.h"

namespace store::detail {

alp::Result<void, io::FileError> publish_directory(const std::filesystem::path& draft_path) {
    const auto context = alp::make_context("renaming {} to its final path", draft_path);
    ASSERT(is_draft(draft_path), draft_path.string());
    const std::filesystem::path draft = paths::normalized(draft_path);
    std::filesystem::path destination = draft;
    destination.replace_extension();
    ALP_TRY_CONTEXT(require_absent(destination), context);
    // The record stays until the rename succeeded, so a draft that failed to publish can still be reused.
    ALP_TRY_CONTEXT(io::filesystem::rename_no_replace(draft, destination), context);
    // A leftover record is harmless, as reuse requires a draft.
    const std::filesystem::path inputs_path = destination / inputs::file_name;
    const auto remove_result = io::filesystem::remove(inputs_path);
    if (!remove_result.has_value()) {
        LOG_WARN("Failed to remove the input record from the published storage: {}", remove_result.error());
    }
    return {};
}

} // namespace store::detail
