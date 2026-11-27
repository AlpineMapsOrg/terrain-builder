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

namespace paths {

// The path with `extension` added after its current extension.
// A leading dot in `extension` is optional.
[[nodiscard]]
inline std::filesystem::path with_added_extension(const std::filesystem::path &path, const std::string_view extension) {
    std::filesystem::path result = path;
    if (!extension.starts_with('.')) {
        result += ".";
    }
    result += extension;
    return result;
}

// The path with its last extension removed.
[[nodiscard]]
inline std::filesystem::path without_extension(std::filesystem::path path) {
    path.replace_extension();
    return path;
}

// Brings `path` into lexically normal form without a trailing separator, so that out/ names out.
inline void normalize(std::filesystem::path &path) {
    path = path.lexically_normal();
    if (!path.has_filename() && path.has_relative_path()) {
        path = path.parent_path();
    }
}

// The path in lexically normal form without a trailing separator, so that out/ names out.
[[nodiscard]]
inline std::filesystem::path normalized(std::filesystem::path path) {
    normalize(path);
    return path;
}

} // namespace paths
