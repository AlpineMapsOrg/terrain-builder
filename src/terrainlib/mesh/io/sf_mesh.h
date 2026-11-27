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

#include <cstdint>
#include <filesystem>
#include <string_view>
#include <vector>

#include <fmt/format.h>

#include "error/MessageCode.h"
#include "error/Result.h"
#include "error/utils.h"
#include "io/PayloadSchema.h"
#include "io/bytes.h"
#include "io/envelope.h"
#include "mesh/SimpleMesh.h"
#include "mesh/encode.h"
#include "mesh/encode_error.h"
#include "mesh/io/options.h"

namespace mesh::io::sf_mesh {

// The extension of SF mesh files.
inline constexpr std::string_view extension = ".sfmesh";

namespace v1 {

    struct Payload {
        std::uint32_t vertex_count;
        std::uint32_t face_count;
        std::vector<std::uint8_t> triangles;
        std::vector<std::uint8_t> positions;
        std::vector<std::uint8_t> uvs;
        std::vector<std::uint8_t> texture;
    };

} // namespace v1

using Schema = ::io::envelope::PayloadSchema<"mesh.SfMesh", ::io::envelope::Version<1, v1::Payload>>;
using Payload = Schema::Latest;

struct CorruptData : alp::MessageCode<"the data is not a valid SF mesh"> {};

inline alp::Result<void, CorruptData> validate(const Payload& payload) {
    if (payload.vertex_count == 0 && (!payload.positions.empty() || !payload.uvs.empty())) {
        return alp::make_error(CorruptData{}, fmt::format("the mesh has no vertices but {} bytes of positions and {} bytes of UVs", payload.positions.size(), payload.uvs.size()));
    }
    if (payload.face_count == 0 && !payload.triangles.empty()) {
        return alp::make_error(CorruptData{}, fmt::format("the mesh has no faces but {} bytes of triangles", payload.triangles.size()));
    }
    return {};
}

using DeserializeError = alp::Error<::io::envelope::DeserializeError, CorruptData, mesh::DecodeError>;
[[nodiscard]]
alp::Result<mesh::Simple, DeserializeError> deserialize(::io::bytes::span bytes, ::io::envelope::DeserializeOptions options = {});

using SerializeError = alp::Error<mesh::EncodeError, ::io::envelope::SerializeError>;
[[nodiscard]]
alp::Result<::io::bytes::vector, SerializeError> serialize(const mesh::Simple& mesh, mesh::EncodeOptions options = {});

using LoadError = alp::Error<::io::bytes::ReadError, DeserializeError>;
[[nodiscard]]
alp::Result<mesh::Simple, LoadError> load_from_path(const std::filesystem::path& path, ::io::envelope::ReadOptions options = {});

using SaveError = alp::Error<mesh::EncodeError, ::io::envelope::WriteError>;
// The result is the hash the file's envelope records.
[[nodiscard]]
alp::Result<::io::bytes::vector, SaveError> save_to_path(const mesh::Simple& mesh, const std::filesystem::path& path, const SaveOptions& options = {});

} // namespace mesh::io::sf_mesh
