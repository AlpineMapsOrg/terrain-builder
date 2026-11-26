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

#include <ostream>
#include <vector>

#include "cli_support/log.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/alloc.h"
#include "io/image/Format.h"
#include "io/mat.h"
#include "mesh/EncodedMesh.h"
#include "mesh/SimpleMesh.h"
#include "mesh/encode_error.h"
#include "meshopt/encode.h"

namespace mesh {

namespace {
constexpr uint32_t GL_BYTE = 0x1400;
constexpr uint32_t GL_UNSIGNED_BYTE = 0x1401;
constexpr uint32_t GL_SHORT = 0x1402;
constexpr uint32_t GL_UNSIGNED_SHORT = 0x1403;
constexpr uint32_t GL_INT = 0x1404;
constexpr uint32_t GL_UNSIGNED_INT = 0x1405;
constexpr uint32_t GL_FLOAT = 0x1406;
constexpr uint32_t GL_DOUBLE = 0x140A;
}

template <typename T>
constexpr uint32_t component_type_id() {
    if constexpr (std::is_same_v<T, float>) {
        return GL_FLOAT;
    } else if constexpr (std::is_same_v<T, double>) {
        return GL_DOUBLE;
    } else if constexpr (std::is_same_v<T, int>) {
        return GL_INT;
    } else if constexpr (std::is_same_v<T, unsigned int>) {
        return GL_UNSIGNED_INT;
    } else if constexpr (std::is_same_v<T, short>) {
        return GL_SHORT;
    } else if constexpr (std::is_same_v<T, unsigned short>) {
        return GL_UNSIGNED_SHORT;
    } else if constexpr (std::is_same_v<T, char>) {
        return GL_BYTE;
    } else if constexpr (std::is_same_v<T, unsigned char>) {
        return GL_UNSIGNED_BYTE;
    } else {
        return -1;
    }
}

struct EncodeOptions {
    ::io::image::Format texture_format = ::io::image::Format::Jpeg;
};

template <glm::length_t n_dims, typename T>
[[nodiscard]]
alp::Result<Encoded, EncodeError> encode(const Simple_<n_dims, T>& mesh, const EncodeOptions options = {}) {
    using Mesh = Simple_<n_dims, T>;

    // Encode positions
    std::vector<uint8_t> position_buf;
    if (mesh.vertex_count() > 0) {
        position_buf = meshopt::encode_vertex_buffer(std::span<const typename Mesh::Position>(mesh.positions));
    }

    // Encode uvs
    std::vector<uint8_t> uv_buf;
    if (mesh.has_uvs()) {
        uv_buf = meshopt::encode_vertex_buffer(std::span<const typename Mesh::Uv>(mesh.uvs));
    }

    // Encode triangles
    std::vector<uint8_t> index_buf;
    if (mesh.face_count() > 0) {
        index_buf = meshopt::encode_index_buffer(std::span<const typename Mesh::Triangle>(mesh.triangles));
    }

    // Encode texture
    std::vector<uint8_t> texture_buf;
    if (mesh.has_texture()) {
        ALP_TRY_CONTEXT(texture_buf, ::io::mat::encode(mesh.texture.value(), options.texture_format), "encoding the texture");
    }

    const Encoded::Header header{
        .version = 1,
        .n_dims = n_dims,
        .component_type = component_type_id<T>(),
        .vertex_count = static_cast<uint32_t>(mesh.vertex_count()),
        .face_count = static_cast<uint32_t>(mesh.face_count())};

    return Encoded{
        .header = header,
        .triangles = index_buf,
        .positions = position_buf,
        .uvs = uv_buf,
        .texture = texture_buf};
}

struct DecodeOptions {
};


template <glm::length_t n_dims = 3, typename T = double>
[[nodiscard]]
alp::Result<Simple_<n_dims, T>, DecodeError> decode(const Encoded &encoded, const DecodeOptions = {}) {
    const uint32_t expected_component_type = component_type_id<T>();
    const Encoded::Header& header = encoded.header;
    if (header.version != 1 ||
        header.n_dims != n_dims ||
        header.component_type != expected_component_type) {
        return alp::make_error(GeometryDecodeError::IncompatibleData);
    }

    using Mesh = Simple_<n_dims, T>;
    Mesh mesh;

    // Decode positions
    const size_t vertex_count = header.vertex_count;
    if (vertex_count > 0) {
        ALP_TRY(::io::try_resize(mesh.positions, vertex_count));
        auto positions_result = meshopt::try_decode_vertex_buffer(std::span(mesh.positions), encoded.positions);
        if (!positions_result) {
            return alp::make_error(GeometryDecodeError::Positions, std::move(positions_result).error());
        }
    }

    // Decode uvs
    if (!encoded.uvs.empty()) {
        ALP_TRY(::io::try_resize(mesh.uvs, vertex_count));
        auto uvs_result = meshopt::try_decode_vertex_buffer(std::span(mesh.uvs), encoded.uvs);
        if (!uvs_result) {
            return alp::make_error(GeometryDecodeError::Uvs, std::move(uvs_result).error());
        }
    }

    // Decode triangles
    const size_t face_count = header.face_count;
    if (face_count > 0) {
        ALP_TRY(::io::try_resize(mesh.triangles, face_count));
        auto triangles_result = meshopt::try_decode_index_buffer(std::span(mesh.triangles), encoded.triangles);
        if (!triangles_result) {
            return alp::make_error(GeometryDecodeError::Triangles, std::move(triangles_result).error());
        }
    }

    // Decode texture
    if (!encoded.texture.empty()) {
        ALP_TRY_CONTEXT(mesh.texture, ::io::mat::decode(encoded.texture, {.mode = cv::IMREAD_UNCHANGED}), "decoding the texture");
    }

    return mesh;
}
}
