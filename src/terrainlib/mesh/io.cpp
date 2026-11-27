/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2023 Martin Braunsperger
 * Copyright (C) 2023 Adam Celarek-Litofcenko
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

#include "mesh/io.h"

#include <string>
#include <string_view>

#include <fmt/format.h>
#include <fmt/std.h>
#include <libassert/assert.hpp>

#include "error/try.h"
#include "io/UnknownExtension.h"
#include "log.h"
#include "mesh/validate/debug_assert.h"
#include "string_utils.h"

namespace mesh::io {

namespace {

    // The mesh formats load_from_path and save_to_path dispatch to.
    enum class Format {
        Gltf,
        SfMesh,
    };

    // The format that `extension`, with its dot, names, ignoring ASCII case.
    alp::Result<Format, Unsupported> format_from_extension(const std::string_view extension) {
        if (gltf::container_from_extension(extension).has_value()) {
            return Format::Gltf;
        }
        if (equals_ignore_ascii_case(extension, sf_mesh::extension)) {
            return Format::SfMesh;
        }
        return ::io::make_unsupported_extension_error(extension, extensions);
    }

    // The format that the extension of `path` names, ignoring ASCII case.
    alp::Result<Format, Unsupported> format_from_path(const std::filesystem::path &path) {
        ALP_TRY_CONTEXT(const Format format, format_from_extension(path.extension().string()), fmt::format("choosing the mesh format for {}", path));
        return format;
    }

} // namespace

alp::Result<SimpleMesh, LoadError> load_from_path(
    const std::filesystem::path &path,
    const LoadOptions& options) {
    ALP_TRY(const Format format, format_from_path(path));
    switch (format) {
    case Format::Gltf: {
        ALP_TRY(SimpleMesh mesh, gltf::load_from_path(path, options));
        return mesh;
    }
    case Format::SfMesh: {
        ALP_TRY(SimpleMesh mesh, sf_mesh::load_from_path(path));
        return mesh;
    }
    }
    UNREACHABLE("invalid mesh format", format);
}

alp::Result<void, SaveError> save_to_path(
    const SimpleMesh &mesh,
    const std::filesystem::path &path,
    const SaveOptions &options) {
    LOG_TRACE("Saving mesh to path {}", path);

    mesh::debug_assert_valid_basic(mesh);

    ALP_TRY(const Format format, format_from_path(path));
    switch (format) {
    case Format::Gltf:
        ALP_TRY(gltf::save_to_path(mesh, path, options));
        return {};
    case Format::SfMesh:
        ALP_TRY(sf_mesh::save_to_path(mesh, path, options));
        return {};
    }
    UNREACHABLE("invalid mesh format", format);
}

}
