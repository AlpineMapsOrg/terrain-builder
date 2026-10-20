#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "error/Result.h"

namespace io::compression {

using Bytes = std::vector<std::uint8_t>;

inline constexpr std::size_t default_max_decompressed_size = std::size_t{1} << 30;

enum class ChecksumAlgorithm : std::uint8_t {
    None,
    HandledByCompressionLib,
    Crc32c,
};

enum class Algorithm : std::uint8_t {
    None,
    ZstdBestCompressionWithChecksum,
};

// A failure to compress or decompress data.
enum class Error : std::uint8_t {
    UnsupportedChecksumAlgorithm,
    UnsupportedAlgorithm,
    InvalidAlgorithmCombination,
    ChecksumMismatch,
    CompressionFailed,
    DecompressionFailed,
    SizeLimitExceeded,
};

inline std::string_view format_as(const Error error) {
    switch (error) {
    case Error::UnsupportedChecksumAlgorithm:
        return "the checksum algorithm is not supported";
    case Error::UnsupportedAlgorithm:
        return "the compression algorithm is not supported";
    case Error::InvalidAlgorithmCombination:
        return "the checksum algorithm does not fit the compression algorithm";
    case Error::ChecksumMismatch:
        return "the checksum does not match the data";
    case Error::CompressionFailed:
        return "compressing the data failed";
    case Error::DecompressionFailed:
        return "decompressing the data failed";
    case Error::SizeLimitExceeded:
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
alp::Result<CompressedData, Error> compress_with_checksum(
    const Bytes &uncompressed_data,
    Algorithm compression_algorithm,
    ChecksumAlgorithm checksum_algorithm);

[[nodiscard]]
alp::Result<Bytes, Error> checked_decompress(
    const Bytes &compressed_data,
    Algorithm compression_algorithm,
    ChecksumAlgorithm checksum_algorithm,
    std::string_view checksum,
    std::size_t max_decompressed_size = default_max_decompressed_size);

} // namespace io::compression
