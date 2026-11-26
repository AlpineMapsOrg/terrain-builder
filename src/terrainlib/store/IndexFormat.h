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
#include <optional>
#include <string_view>
#include <utility>

#include "common_error_codes.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "io/FileError.h"
#include "io/alloc.h"
#include "store/Index.h"
#include "store/path_layout.h"

namespace store {

struct UnencodableIndex : alp::MessageCode<"the storage index cannot be encoded"> {};

using ReadIndexError = alp::Error<io::FileError, io::AllocError, CorruptIndex, Unsupported>;
using WriteIndexError = alp::Error<io::FileError, io::AllocError, UnencodableIndex>;

// How a family of storages reads and writes its index, and which node layouts it has.
template <Index I>
struct IndexFormat {
    using Key = typename I::Traits::Key;

    // Reads the index of the storage at `base_path`.
    alp::Result<I, ReadIndexError> (*read)(const std::filesystem::path& base_path);
    // Writes the index of the storage at `base_path`, into files inside that directory.
    alp::Result<void, WriteIndexError> (*write)(const std::filesystem::path& base_path, const I& index);
    std::optional<path_layout::Layout<Key>> (*resolve_layout)(std::string_view id);
    path_layout::Layout<Key> (*default_layout)();
};

} // namespace store
