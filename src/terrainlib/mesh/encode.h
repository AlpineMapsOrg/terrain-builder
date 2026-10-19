#pragma once

#include <ostream>
#include <vector>

#include <meshoptimizer.h>

#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/UnsupportedFormat.h"
#include "log.h"
#include "mesh/EncodedMesh.h"
#include "mesh/SimpleMesh.h"
#include "mesh/encode_error.h"
#include "texture/io.h"

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
    // Texture format
    std::string texture_format = ".jpeg";
};

template <glm::length_t n_dims, typename T>
[[nodiscard]]
alp::Result<Encoded, EncodeError> encode(const Simple_<n_dims, T>& mesh, const EncodeOptions options = {}) {
    using Mesh = Simple_<n_dims, T>;

    const size_t vertex_count = mesh.vertex_count();
    const size_t position_size = sizeof(typename Mesh::Position);
    const size_t uv_size = sizeof(typename Mesh::Uv);
    const size_t face_count = mesh.face_count();
    const size_t index_count = face_count * 3;

    // Encode positions
    std::vector<uint8_t> position_buf;
    if (vertex_count > 0) {
        position_buf.resize(meshopt_encodeVertexBufferBound(vertex_count, position_size));
        const size_t pos_written = meshopt_encodeVertexBuffer(position_buf.data(), position_buf.size(), mesh.positions.data(), vertex_count, position_size);
        if (pos_written == 0) {
            return alp::make_unexpected(GeometryEncodeError::Positions);
        }
        position_buf.resize(pos_written);
    }

    // Encode uvs
    std::vector<uint8_t> uv_buf;
    if (mesh.has_uvs()) {
        uv_buf.resize(meshopt_encodeVertexBufferBound(vertex_count, uv_size));
        const size_t uv_written = meshopt_encodeVertexBuffer(uv_buf.data(), uv_buf.size(), mesh.uvs.data(), vertex_count, uv_size);
        if (uv_written == 0) {
            return alp::make_unexpected(GeometryEncodeError::Uvs);
        }
        uv_buf.resize(uv_written);
    }

    // Encode triangles
    std::vector<uint8_t> index_buf;
    if (face_count > 0) {
        index_buf.resize(meshopt_encodeIndexBufferBound(index_count, vertex_count));
        const size_t index_written = meshopt_encodeIndexBuffer(
            index_buf.data(), index_buf.size(),
            reinterpret_cast<const unsigned int *>(mesh.triangles.data()),
            index_count);
        if (index_written == 0) {
            return alp::make_unexpected(GeometryEncodeError::Triangles);
        }
        index_buf.resize(index_written);
    }

    // Encode texture
    std::vector<uint8_t> texture_buf;
    if (mesh.has_texture()) {
        ALP_TRY_CONTEXT(texture_buf, texture::io::encode_to_bytes(mesh.texture.value(), options.texture_format), "encoding the texture");
    }

    const Encoded::Header header{
        .version = 1,
        .n_dims = n_dims,
        .component_type = component_type_id<T>(),
        .vertex_count = static_cast<uint32_t>(vertex_count),
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
        return alp::make_unexpected(GeometryDecodeError::IncompatibleData);
    }

    using Mesh = Simple_<n_dims, T>;
    Mesh mesh;
    int result;

    // Decode positions
    const size_t position_size = sizeof(typename Mesh::Position);
    const size_t vertex_count = header.vertex_count;
    if (vertex_count > 0) {
        mesh.positions.resize(vertex_count);

        result = meshopt_decodeVertexBuffer(mesh.positions.data(), vertex_count, position_size, encoded.positions.data(), encoded.positions.size());
        if (result != 0) {
            return alp::make_unexpected(GeometryDecodeError::Positions);
        }
    }

    // Decode uvs
    if (!encoded.uvs.empty()) {
        const size_t uv_size = sizeof(typename Mesh::Uv);
        mesh.uvs.resize(vertex_count);
        result = meshopt_decodeVertexBuffer(mesh.uvs.data(), vertex_count, uv_size, encoded.uvs.data(), encoded.uvs.size());
        if (result != 0) {
            return alp::make_unexpected(GeometryDecodeError::Uvs);
        }
    }

    // Decode triangles
    const size_t face_count = header.face_count;
    const size_t index_count = face_count * 3;
    const size_t index_size = sizeof(typename Mesh::Triangle::value_type);
    if (face_count > 0) {
        mesh.triangles.resize(face_count);
        
        result = meshopt_decodeIndexBuffer(mesh.triangles.data(), index_count, index_size, encoded.triangles.data(), encoded.triangles.size());
        if (result != 0) {
            return alp::make_unexpected(GeometryDecodeError::Triangles);
        }
    }

    // Decode texture
    if (!encoded.texture.empty()) {
        ALP_TRY_CONTEXT(mesh.texture, texture::io::decode_bytes(encoded.texture, cv::IMREAD_UNCHANGED), "decoding the texture");
    }

    return mesh;
}
}
