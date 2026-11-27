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

#include <filesystem>
#include <utility>

#include <fmt/std.h>

#include "error/Context.h"
#include "error/Error.h"
#include "error/Result.h"
#include "error/try.h"
#include "mesh/storage.h"
#include "sf/validate_index.h"
#include "store/IndexFormat.h"
#include "store/publish.h"

namespace sf {

using PublishError = alp::Error<store::WriteIndexError, UnexpectedInnerNode, store::PublishError>;
// Saves the draft's index, checks that it has the SF topology, and publishes it.
// An index that fails the check stays saved in the draft, which is not published.
[[nodiscard]]
inline alp::Result<void, PublishError> publish(mesh::storage::Draft draft) {
    // The context holds a reference, and `draft` is moved into mesh::storage::publish.
    const std::filesystem::path base_path = draft.base_path();
    const auto context = alp::make_context("publishing the SF storage at {}", base_path);

    ALP_TRY_CONTEXT(draft.save_index(), context);
    ALP_TRY_CONTEXT(validate_index(draft.index()), context);
    ALP_TRY_CONTEXT(mesh::storage::publish(std::move(draft)), context);
    return {};
}

} // namespace sf
