/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2024 Martin Braunsperger
 * Copyright (C) 2024 Adam Celarek-Litofcenko
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

#include <algorithm>
#include <array>
#include <filesystem>
#include <string_view>

#include "common_error_codes.h"
#include "error/Error.h"
#include "error/Result.h"
#include "mesh/SimpleMesh.h"
#include "mesh/io/gltf.h"
#include "mesh/io/options.h"
#include "mesh/io/sf_mesh.h"

namespace mesh::io {

// The extensions load_from_path and save_to_path recognise.
inline constexpr auto extensions = [] {
    std::array<std::string_view, gltf::extensions.size() + 1> result;
    std::ranges::copy(gltf::extensions, result.begin());
    result.back() = sf_mesh::extension;
    return result;
}();

using LoadError = alp::Error<Unsupported, gltf::LoadError, sf_mesh::LoadError>;
[[nodiscard]]
alp::Result<SimpleMesh, LoadError> load_from_path(
    const std::filesystem::path &path,
    const LoadOptions& options = {});

using SaveError = alp::Error<Unsupported, gltf::SaveError, sf_mesh::SaveError>;
[[nodiscard]]
alp::Result<void, SaveError> save_to_path(
    const SimpleMesh &mesh,
    const std::filesystem::path &path,
    const SaveOptions& options = {});

}
