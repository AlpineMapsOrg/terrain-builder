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
#include <string_view>

#include "error/MessageCode.h"

namespace store {

struct InvalidStoragePath : alp::MessageCode<"the path cannot name a storage"> {};

// The suffix that marks a draft's directory.
inline constexpr std::string_view draft_suffix = ".part";

// The path of the draft that becomes the storage at `final_path`: `<final_path>.part`.
std::filesystem::path make_draft_path(const std::filesystem::path& final_path);

// Whether `path` names a draft.
bool is_draft(const std::filesystem::path& path);

} // namespace store
