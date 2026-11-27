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

#define ZSTD_STATIC_LINKING_ONLY
#include <zstd.h>
#include <zstd_errors.h>

#include "io/compression.h"

#include <algorithm>

#include <fmt/format.h>
#include <libassert/assert.hpp>

#include "error/try.h"
#include "error/utils.h"
#include "io/ZstdError.h"
#include "io/alloc.h"

namespace io::compression {
namespace {

// The decompression error for a code Zstandard reported, created from that code.
DecompressError to_decompress_error(const ZstdError error) {
    switch (error.code()) {
    case ZSTD_error_memory_allocation:
        return alp::make_error(OutOfMemory{}, error);
    case ZSTD_error_prefix_unknown:
    case ZSTD_error_corruption_detected:
    case ZSTD_error_checksum_wrong:
    case ZSTD_error_literals_headerWrong:
    case ZSTD_error_tableLog_tooLarge:
    case ZSTD_error_maxSymbolValue_tooLarge:
    case ZSTD_error_maxSymbolValue_tooSmall:
    case ZSTD_error_srcSize_wrong:
    // No dictionary is ever given, so the corrupt data is the frame's own.
    case ZSTD_error_dictionary_corrupted:
    // The buffer holds the bound, which only corrupt data can exceed.
    case ZSTD_error_dstSize_tooSmall:
        return alp::make_error(CorruptData{}, error);
    case ZSTD_error_version_unsupported:
    case ZSTD_error_frameParameter_unsupported:
    case ZSTD_error_frameParameter_windowTooLarge:
    // The frame needs a dictionary, and no dictionary is ever given.
    case ZSTD_error_dictionary_wrong:
        return alp::make_error(Unsupported{}, error);
    default:
        return alp::make_error(Unknown{}, error);
    }
}

alp::Result<bytes::vector, CompressError> compress_with_zstd(const bytes::span uncompressed_data, const int compression_level) {
    constexpr auto context = "compressing with Zstandard";

    ALP_TRY_CONTEXT(bytes::vector compressed_data, try_make_vector<uint8_t>(ZSTD_compressBound(uncompressed_data.size())), context);
    const size_t compress_result = ZSTD_compress(compressed_data.data(), compressed_data.size(), uncompressed_data.data(), uncompressed_data.size(), compression_level);
    const auto compressed_size_result = to_zstd_result(compress_result);
    if (!compressed_size_result) {
        const ZstdError error = compressed_size_result.error();
        if (error.code() == ZSTD_error_memory_allocation) {
            return alp::make_error(OutOfMemory{}, context, error);
        }
        return alp::make_error(Unknown{}, context, error);
    }
    compressed_data.resize(*compressed_size_result);
    return compressed_data;
}

alp::Result<bytes::vector, DecompressError> decompress_with_zstd(const bytes::span compressed_data, const size_t max_size) {
    constexpr auto context = "decompressing Zstandard data";

    ZSTD_frameHeader frame_header{};
    const size_t get_frame_header_result = ZSTD_getFrameHeader(&frame_header, compressed_data.data(), compressed_data.size());
    const auto frame_header_result = to_zstd_result(get_frame_header_result);
    if (!frame_header_result) {
        return to_decompress_error(frame_header_result.error()).context(context);
    }
    // A positive result is the input size the header needs, so the header is incomplete.
    if (*frame_header_result != 0) {
        return alp::make_error(Truncated{}, context);
    }

    const unsigned long long declared_size = ZSTD_findDecompressedSize(compressed_data.data(), compressed_data.size());
    // Equals the declared size when every frame declares its size, so only data without one can exceed the limit below.
    const unsigned long long decompressed_bound = ZSTD_decompressBound(compressed_data.data(), compressed_data.size());
    if (declared_size == ZSTD_CONTENTSIZE_ERROR || decompressed_bound == ZSTD_CONTENTSIZE_ERROR) {
        return alp::make_error(CorruptData{}, context);
    }
    const bool size_is_declared = declared_size != ZSTD_CONTENTSIZE_UNKNOWN;
    if (size_is_declared && declared_size > max_size) {
        return alp::make_error(BufferLimitExceeded(max_size, declared_size), context);
    }
    const bool buffer_is_limited = decompressed_bound > max_size;
    const size_t allocation_size = buffer_is_limited ? max_size : static_cast<size_t>(decompressed_bound);

    ALP_TRY_CONTEXT(bytes::vector uncompressed_data, try_make_vector<uint8_t>(allocation_size), context);
    const size_t decompress_result = ZSTD_decompress(uncompressed_data.data(), uncompressed_data.size(), compressed_data.data(), compressed_data.size());
    const auto decompressed_size_result = to_zstd_result(decompress_result);
    if (!decompressed_size_result) {
        const ZstdError error = decompressed_size_result.error();
        // A buffer cut to the limit can be too small for valid data.
        if (error.code() == ZSTD_error_dstSize_tooSmall && buffer_is_limited) {
            return alp::make_error(BufferLimitExceeded(max_size), error)
                .context("the data does not declare its size")
                .context(context);
        }
        return to_decompress_error(error).context(context);
    }
    if (size_is_declared && *decompressed_size_result != declared_size) {
        return alp::make_error(CorruptData{})
            .context(fmt::format("the data decodes to {} bytes, not the {} bytes it declares", *decompressed_size_result, declared_size))
            .context(context);
    }

    uncompressed_data.resize(*decompressed_size_result);
    return uncompressed_data;
}

} // namespace

alp::Result<bytes::vector, CompressError> compress(const bytes::span uncompressed_data, const Algorithm compression_algorithm) {
    if (uncompressed_data.size() > max_buffer_size) {
        return alp::make_error(BufferLimitExceeded(max_buffer_size, uncompressed_data.size()), "compressing");
    }

    switch (compression_algorithm) {
    case Algorithm::None: {
        ALP_TRY_CONTEXT(bytes::vector copy, try_to_vector(uncompressed_data), "copying data stored without compression");
        return copy;
    }
    case Algorithm::ZstdBest:
        return compress_with_zstd(uncompressed_data, ZSTD_maxCLevel());
    case Algorithm::ZstdDefault:
        return compress_with_zstd(uncompressed_data, ZSTD_defaultCLevel());
    }
    PANIC("unsupported compression algorithm", compression_algorithm);
}

alp::Result<bytes::vector, DecompressError> decompress(
    const bytes::span compressed_data, const Algorithm compression_algorithm, const size_t max_decompressed_size) {
    const size_t max_size = std::min(max_decompressed_size, max_buffer_size);

    switch (compression_algorithm) {
    case Algorithm::None: {
        ALP_TRY_CONTEXT(bytes::vector copy, try_to_vector(compressed_data, max_size), "decompressing data stored without compression");
        return copy;
    }
    case Algorithm::ZstdBest:
    case Algorithm::ZstdDefault:
        return decompress_with_zstd(compressed_data, max_size);
    }
    PANIC("unsupported compression algorithm", compression_algorithm);
}

} // namespace io::compression
