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

#include <system_error>

#include "error/try.h"
#include "error/utils.h"
#include "io/filesystem.h"
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

alp::Result<void, io::FileError> require_absent(const std::filesystem::path& path) {
    const io::FileError exists_error(std::errc::file_exists, path);
    ALP_TRY(const bool symlink, io::filesystem::is_symlink(path));
    if (symlink) {
        return alp::make_error(exists_error, "the path is a symlink");
    }
    ALP_TRY(const bool exists, io::filesystem::exists(path));
    if (exists) {
        return alp::make_error(exists_error);
    }
    return {};
}

} // namespace store
