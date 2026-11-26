/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2025 Martin Braunsperger
 * Copyright (C) 2025 Adam Celarek-Litofcenko
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

#include <array>
#include <filesystem>
#include <string_view>

#include <libassert/assert.hpp>
#include <magic_enum/magic_enum.hpp>

#include "array_utils.h"
#include "common_error_codes.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "io/alloc.h"
#include "io/bytes.h"
#include "io/mat.h"
#include "mesh/SimpleMesh.h"
#include "mesh/io/options.h"

namespace mesh::io::gltf {

// How a glTF file packages its data.
enum class Container {
    Binary,
    Json,
};

// The extension, with its dot, that names `container`.
constexpr std::string_view to_extension(const Container container) {
    switch (container) {
    case Container::Binary:
        return ".glb";
    case Container::Json:
        return ".gltf";
    }
    UNREACHABLE("invalid glTF container", container);
}

// The extensions container_from_extension recognises.
inline constexpr auto extensions = transform_array(magic_enum::enum_values<Container>(), to_extension);

// The container that `extension`, with its dot, names, ignoring ASCII case.
[[nodiscard]]
alp::Result<Container, Unsupported> container_from_extension(std::string_view extension);

// The container that the extension of `path` names, ignoring ASCII case.
[[nodiscard]]
alp::Result<Container, Unsupported> container_from_path(const std::filesystem::path &path);

struct CorruptData : alp::MessageCode<"the data is not a valid glTF file"> {};

using LoadError = alp::Error<::io::bytes::ReadError, CorruptData, Unsupported, OutOfMemory, ::io::mat::DecodeError>;
[[nodiscard]]
alp::Result<SimpleMesh, LoadError> load_from_path(
    const std::filesystem::path &path,
    const LoadOptions &options = {});

using SaveError = alp::Error<Unsupported, ::io::mat::EncodeError, ::io::AllocError, ::io::bytes::WriteError>;
// Writes `mesh` to the glTF file at `path`, in the container its extension names.
[[nodiscard]]
alp::Result<void, SaveError> save_to_path(
    const SimpleMesh &mesh,
    const std::filesystem::path &path,
    const SaveOptions &options = {});

} // namespace mesh::io::gltf
