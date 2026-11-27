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

#include <fmt/std.h>

#include "error/Context.h"
#include "error/Error.h"
#include "error/Result.h"
#include "error/try.h"
#include "mesh/storage.h"
#include "sf/validate_index.h"

namespace sf {

using OpenError = alp::Error<mesh::storage::OpenError, UnexpectedInnerNode>;
// Opens the SF storage at `path`.
[[nodiscard]]
inline alp::Result<mesh::storage::Storage, OpenError> open(const std::filesystem::path& path) {
    const auto context = alp::make_context("opening the SF storage at {}", path);
    ALP_TRY_CONTEXT(mesh::storage::Storage storage, mesh::storage::open(path), context);
    ALP_TRY_CONTEXT(validate_index(storage.index()), context);
    return storage;
}

} // namespace sf
