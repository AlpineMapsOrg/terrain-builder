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

#include "common_error_codes.h"
#include "error/Error.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "io/alloc.h"
#include "io/bytes.h"
#include "io/limits.h"

namespace io::compression {

// How data is compressed.
enum class Algorithm : uint8_t {
    None,
    ZstdBest,
    ZstdDefault,
};

struct CorruptData : alp::MessageCode<"the input data is not valid for the algorithm"> {};
struct Truncated : alp::MessageCode<"the input data ends inside a frame header"> {};
struct Unknown : alp::MessageCode<"the compression library reported a failure of unknown meaning"> {};

using CompressError = alp::Error<AllocError, Unknown>;

// Produces a compact form of the input data, which `decompress` restores.
// Fails if the input data, or the buffer Zstandard needs for the compressed data, would be larger than `max_buffer_size`.
[[nodiscard]]
alp::Result<bytes::vector, CompressError> compress(bytes::span uncompressed_data, Algorithm compression_algorithm);

using DecompressError = alp::Error<AllocError, CorruptData, Truncated, Unsupported, Unknown>;

// Restores the original data from the form `compress` produced.
// Fails if it would be larger than `max_decompressed_size` or `max_buffer_size`.
[[nodiscard]]
alp::Result<bytes::vector, DecompressError> decompress(
    bytes::span compressed_data, Algorithm compression_algorithm, size_t max_decompressed_size = max_buffer_size);

} // namespace io::compression
