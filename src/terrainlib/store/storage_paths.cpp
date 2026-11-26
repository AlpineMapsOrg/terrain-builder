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

#include "store/storage_paths.h"

#include "path_utils.h"

namespace store {

std::filesystem::path make_draft_path(const std::filesystem::path& final_path) {
    auto path = paths::normalized(final_path);
    path += draft_suffix;
    return path;
}

bool is_draft(const std::filesystem::path& path) {
    return paths::normalized(path).extension() == draft_suffix;
}

} // namespace store
