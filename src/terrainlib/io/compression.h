#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "error/Result.h"

namespace io::envelope {

using Bytes = std::vector<std::byte>;

inline constexpr std::size_t default_max_decompressed_size = std::size_t{1} << 30;

enum class ChecksumAlgorithm : std::uint8_t {
    None,
    HandledByCompressionLib,
    Crc32c,
};

enum class CompressionAlgorithm : std::uint8_t {
    None,
    ZstdBestCompressionWithChecksum,
};

// A failure to compress or decompress data.
enum class CompressionError : std::uint8_t {
    UnsupportedChecksumAlgorithm,
    UnsupportedCompressionAlgorithm,
    InvalidAlgorithmCombination,
    ChecksumMismatch,
    CompressionFailed,
    DecompressionFailed,
    SizeLimitExceeded,
};

inline std::string_view format_as(const CompressionError error) {
    switch (error) {
    case CompressionError::UnsupportedChecksumAlgorithm:
        return "the checksum algorithm is not supported";
    case CompressionError::UnsupportedCompressionAlgorithm:
        return "the compression algorithm is not supported";
    case CompressionError::InvalidAlgorithmCombination:
        return "the checksum algorithm does not fit the compression algorithm";
    case CompressionError::ChecksumMismatch:
        return "the checksum does not match the data";
    case CompressionError::CompressionFailed:
        return "compressing the data failed";
    case CompressionError::DecompressionFailed:
        return "decompressing the data failed";
    case CompressionError::SizeLimitExceeded:
        return "the data is larger than the size limit";
    default:
        return "an unknown compression failure";
    }
}

struct CompressedData {
    Bytes compressed_data;
    std::string checksum;
};

[[nodiscard]]
alp::Result<CompressedData, CompressionError> compress_with_checksum(
    const Bytes &uncompressed_data,
    CompressionAlgorithm compression_algorithm,
    ChecksumAlgorithm checksum_algorithm);

[[nodiscard]]
alp::Result<Bytes, CompressionError> checked_decompress(
    const Bytes &compressed_data,
    CompressionAlgorithm compression_algorithm,
    ChecksumAlgorithm checksum_algorithm,
    std::string_view checksum,
    std::size_t max_decompressed_size = default_max_decompressed_size);

} // namespace io::envelope
