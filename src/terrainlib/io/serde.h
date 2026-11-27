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

#include <cstdint>
#include <filesystem>

#include <zpp_bits.h>

#include "common_error_codes.h"
#include "error/Error.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "io/alloc.h"
#include "io/bytes.h"
#include "io/limits.h"

namespace io::serde {

struct Unserializable : alp::MessageCode<"the value has no encoding"> {};
struct CorruptData : alp::MessageCode<"the data is no encoding of the value"> {};
struct Truncated : alp::MessageCode<"the data ends inside the value"> {};
struct TrailingData : alp::MessageCode<"the data continues after the value"> {};

using SerializeError = alp::Error<AllocError, Unserializable>;
// Encodes `value` as bytes in little-endian byte order.
// Fails if the value holds a null pointer or a variant without a value, or if the encoding outgrows `max_buffer_size`.
template <typename T>
[[nodiscard]]
alp::Result<bytes::vector, SerializeError> serialize(const T &value);

// A value at the start of encoded data, and the bytes after it.
template <typename T>
struct Prefix {
    T value;
    // Points into the data the value was decoded from.
    zpp::bits::static_span<const uint8_t> rest;
};

using DeserializePrefixError = alp::Error<AllocError, CorruptData, Truncated>;
// Decodes the value encoded at the start of `bytes`.
// Fails if a container in it would be larger than `MaxSize`.
template <typename T, size_t MaxSize = max_buffer_size>
[[nodiscard]]
alp::Result<Prefix<T>, DeserializePrefixError> deserialize_prefix(bytes::span bytes);

using DeserializeError = alp::Error<DeserializePrefixError, TrailingData>;
// TODO: reevaluate letting T declare its limit as a `static constexpr` or through a max_buffer_size<T> specialization or similar.
// Decodes the value whose encoding `bytes` holds, with nothing before or after it.
// Fails if a container in it would be larger than `MaxSize`.
template <typename T, size_t MaxSize = max_buffer_size>
[[nodiscard]]
alp::Result<T, DeserializeError> deserialize(bytes::span bytes);

using WriteOptions = bytes::WriteOptions;

using WriteError = alp::Error<bytes::WriteError, SerializeError>;
// Writes `value`, encoded as bytes, to the file at `path`.
template <typename T>
[[nodiscard]]
alp::Result<void, WriteError> write(const T &value, const std::filesystem::path &path, WriteOptions options = {});

using ReadError = alp::Error<bytes::ReadError, DeserializeError>;
// Reads the value encoded in the file at `path`.
template <typename T>
[[nodiscard]]
alp::Result<T, ReadError> read(const std::filesystem::path &path);

using Error = alp::Error<ReadError, WriteError>;

} // namespace io::serde

#include "io/serde.inl"
