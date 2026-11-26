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

#include "store/inputs.h"

#include <system_error>

#include <fmt/std.h>

#include "error/try.h"
#include "error/utils.h"
#include "io/filesystem.h"

namespace store::inputs::detail {

alp::Result<void, io::FileError> require_same_filesystem(const std::filesystem::path& source, const std::filesystem::path& output) {
    const auto context = alp::make_context("checking whether {} can be hard linked into {}", source, output);

    // The output may not exist yet, so its closest existing ancestor decides its filesystem.
    ALP_TRY_CONTEXT(const std::filesystem::path absolute_output, io::filesystem::absolute(output), context);
    std::filesystem::path parent = absolute_output.parent_path();
    ALP_TRY_CONTEXT(bool parent_exists, io::filesystem::exists(parent), context);
    while (!parent_exists) {
        // A root is its own parent, so nothing up to the root exists.
        if (parent == parent.parent_path()) {
            return alp::make_error(io::FileError(std::errc::no_such_file_or_directory, parent)).context(context);
        }
        parent = parent.parent_path();
        ALP_TRY_CONTEXT(parent_exists, io::filesystem::exists(parent), context);
    }

    ALP_TRY_CONTEXT(const bool same_filesystem, io::filesystem::on_same_filesystem(source, parent), context);
    if (!same_filesystem) {
        return alp::make_error(io::FileError(std::errc::cross_device_link, output)).context(context);
    }
    return {};
}

} // namespace store::inputs::detail
