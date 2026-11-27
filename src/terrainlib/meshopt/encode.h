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

#include <algorithm>
#include <concepts>
#include <cstdint>
#include <span>
#include <string>
#include <type_traits>
#include <vector>

#include <fmt/format.h>
#include <glm/glm.hpp>
#include <libassert/assert.hpp>
#include <meshoptimizer.h>

#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "glm_utils.h"

namespace meshopt {

// A decoder's failure, carrying the code meshopt returned.
struct DecodeError {
    int code = 0;

    auto operator<=>(const DecodeError &) const = default;
};

inline std::string format_as(const DecodeError error) {
    // meshopt only documents that a nonzero code is a failure; these meanings are read from the v1.2 decoders.
    switch (error.code) {
    case -1:
        return "the header is not this codec's or its version is not supported";
    case -2:
        return "the buffer is too short";
    case -3:
        return "the buffer is longer than the data it encodes";
    default:
        return fmt::format("an unknown meshopt failure with code {}", error.code);
    }
}

// The level `meshopt_encodeVertexBuffer` encodes at.
inline constexpr int DEFAULT_VERTEX_ENCODE_LEVEL = 2;
// Defers to the version set through `meshopt_encodeVertexVersion`.
inline constexpr int DEFAULT_VERTEX_ENCODE_VERSION = -1;

// An index type the meshopt decoders write natively.
template <typename T>
concept DecodableIndex = std::unsigned_integral<T> && (sizeof(T) == 2 || sizeof(T) == 4);

// Encodes a vertex buffer with meshopt's vertex codec.
template <typename T>
[[nodiscard]] inline std::vector<uint8_t> encode_vertex_buffer(
    const std::span<const T> data,
    const int level = DEFAULT_VERTEX_ENCODE_LEVEL,
    const int version = DEFAULT_VERTEX_ENCODE_VERSION) {
    static_assert(std::is_trivially_copyable_v<T>, "Vertex type must be trivially copyable");

    const size_t bound = meshopt_encodeVertexBufferBound(data.size(), sizeof(T));
    std::vector<uint8_t> buf(bound);
    const size_t written = meshopt_encodeVertexBufferLevel(
        buf.data(),
        buf.size(),
        data.data(),
        data.size(),
        sizeof(T),
        level,
        version);
    ASSERT(written != 0u);

    buf.resize(written);
    return buf;
}
template <typename T>
[[nodiscard]] inline std::vector<uint8_t> encode_vertex_buffer(const std::vector<T>& data) {
    return encode_vertex_buffer(std::span<const T>(data));
}

// Decodes a vertex buffer into `data`, which must hold exactly as many vertices as were encoded.
template <typename T>
[[nodiscard]] inline alp::Result<void, DecodeError> try_decode_vertex_buffer(std::span<T> data, const std::span<const uint8_t> encoded) {
    static_assert(std::is_trivially_copyable_v<T>, "Vertex type must be trivially copyable");

    const int code = meshopt_decodeVertexBuffer(
        data.data(),
        data.size(),
        sizeof(T),
        encoded.data(),
        encoded.size());
    if (code != 0) {
        return alp::make_unexpected(DecodeError{code});
    }
    return {};
}
// Decodes the `vertex_count` vertices held by `encoded` into a new vector.
template <typename T>
[[nodiscard]] inline alp::Result<std::vector<T>, DecodeError> try_decode_vertex_buffer(const size_t vertex_count, const std::span<const uint8_t> encoded) {
    std::vector<T> data(vertex_count);
    ALP_TRY(try_decode_vertex_buffer(std::span<T>(data), encoded));
    return data;
}
// Decodes a vertex buffer into `data`, which must hold exactly as many vertices as were encoded; asserts that it decodes.
template <typename T>
inline void decode_vertex_buffer(std::span<T> data, const std::span<const uint8_t> encoded) {
    const alp::Result<void, DecodeError> result = try_decode_vertex_buffer(data, encoded);
    ASSERT(result.has_value(), "Vertex buffer decoding failed");
}
// Replaces the contents of `data` with the decoded vertices.
template <typename T>
inline void decode_vertex_buffer(std::vector<T>& data, size_t vertex_count, const std::span<const uint8_t> encoded) {
    data.resize(vertex_count);
    decode_vertex_buffer(std::span<T>(data), encoded);
}
// Decodes a vertex buffer into a new vector.
template <typename T>
[[nodiscard]] inline std::vector<T> decode_vertex_buffer(size_t vertex_count, const std::span<const uint8_t> encoded) {
    alp::Result<std::vector<T>, DecodeError> result = try_decode_vertex_buffer<T>(vertex_count, encoded);
    ASSERT(result.has_value(), "Vertex buffer decoding failed");
    return *std::move(result);
}

// Encodes a triangle list with meshopt's index codec.
[[nodiscard]] inline std::vector<uint8_t> encode_index_buffer(const std::span<const uint32_t> indices) {
    const size_t index_count = indices.size();
    const size_t vertex_count = indices.empty() ? 0 : size_t{std::ranges::max(indices)} + 1;
    const size_t bound = meshopt_encodeIndexBufferBound(index_count, vertex_count);
    std::vector<uint8_t> buf(bound);
    const size_t written = meshopt_encodeIndexBuffer(buf.data(), buf.size(), indices.data(), index_count);
    ASSERT(written != 0u);
    buf.resize(written);
    return buf;
}
[[nodiscard]] inline std::vector<uint8_t> encode_index_buffer(const std::span<const glm::uvec3> triangles) {
    std::span<const uint32_t> indices = flatten(triangles);
    return encode_index_buffer(indices);
}

// Decodes a triangle list into `indices`, which must hold exactly as many indices as were encoded.
template <DecodableIndex T>
[[nodiscard]] inline alp::Result<void, DecodeError> try_decode_index_buffer(const std::span<T> indices, const std::span<const uint8_t> encoded) {
    const int code = meshopt_decodeIndexBuffer(
        indices.data(),
        indices.size(),
        sizeof(T),
        encoded.data(),
        encoded.size());
    if (code != 0) {
        return alp::make_unexpected(DecodeError{code});
    }
    return {};
}
// Decodes a triangle list into `triangles`, which must hold exactly as many triangles as were encoded.
template <DecodableIndex T>
[[nodiscard]] inline alp::Result<void, DecodeError> try_decode_index_buffer(const std::span<glm::vec<3, T>> triangles, const std::span<const uint8_t> encoded) {
    const std::span<T> indices = flatten(triangles);
    return try_decode_index_buffer(indices, encoded);
}
// Decodes the `triangle_count` triangles held by `encoded` into a new vector.
template <DecodableIndex T = uint32_t>
[[nodiscard]] inline alp::Result<std::vector<glm::vec<3, T>>, DecodeError> try_decode_index_buffer(const size_t triangle_count, const std::span<const uint8_t> encoded) {
    std::vector<glm::vec<3, T>> triangles(triangle_count);
    ALP_TRY(try_decode_index_buffer(std::span<glm::vec<3, T>>(triangles), encoded));
    return triangles;
}

// Decodes a triangle list into `indices`, which must hold exactly as many indices as were encoded; asserts that it decodes.
template <DecodableIndex T>
inline void decode_index_buffer(const std::span<T> indices, const std::span<const uint8_t> encoded) {
    const alp::Result<void, DecodeError> result = try_decode_index_buffer(indices, encoded);
    ASSERT(result.has_value(), "Index buffer decoding failed");
}
// Decodes a triangle list into `triangles`, which must hold exactly as many triangles as were encoded; asserts that it decodes.
template <DecodableIndex T>
inline void decode_index_buffer(const std::span<glm::vec<3, T>> triangles, const std::span<const uint8_t> encoded) {
    const alp::Result<void, DecodeError> result = try_decode_index_buffer(triangles, encoded);
    ASSERT(result.has_value(), "Index buffer decoding failed");
}
// Replaces the contents of `triangles` with the decoded triangles.
template <DecodableIndex T>
inline void decode_index_buffer(std::vector<glm::vec<3, T>>& triangles, const size_t triangle_count, const std::span<const uint8_t> encoded) {
    triangles.resize(triangle_count);
    decode_index_buffer(std::span<glm::vec<3, T>>(triangles), encoded);
}
// Decodes a triangle list into a new vector.
template <DecodableIndex T = uint32_t>
[[nodiscard]] inline std::vector<glm::vec<3, T>> decode_index_buffer(const size_t triangle_count, const std::span<const uint8_t> encoded) {
    alp::Result<std::vector<glm::vec<3, T>>, DecodeError> result = try_decode_index_buffer<T>(triangle_count, encoded);
    ASSERT(result.has_value(), "Index buffer decoding failed");
    return *std::move(result);
}

} // namespace meshopt