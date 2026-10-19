#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <cstddef>
#include <map>
#include <memory>
#include <string_view>

#include <zstd.h>

#include "../result_helpers.h"
#include "io/compression.h"

namespace {

using io::envelope::Bytes;
using io::envelope::ChecksumAlgorithm;
using io::envelope::CompressionAlgorithm;
using io::envelope::CompressionError;

Bytes bytes_from_string(const std::string_view text) {
    Bytes bytes;
    for (const char character : text) {
        bytes.push_back(static_cast<std::byte>(character));
    }
    return bytes;
}

// Compresses `data` into a zstd frame, with `parameters` set on the compression context.
Bytes compress_with_zstd(const Bytes &data, const std::map<ZSTD_cParameter, int> &parameters) {
    const std::unique_ptr<ZSTD_CCtx, decltype(&ZSTD_freeCCtx)> context{ZSTD_createCCtx(), &ZSTD_freeCCtx};
    REQUIRE(context);
    for (const auto &[parameter, value] : parameters) {
        REQUIRE_FALSE(ZSTD_isError(ZSTD_CCtx_setParameter(context.get(), parameter, value)));
    }

    Bytes compressed_data(ZSTD_compressBound(data.size()));
    const size_t compressed_size = ZSTD_compress2(context.get(), compressed_data.data(), compressed_data.size(), data.data(), data.size());
    REQUIRE_FALSE(ZSTD_isError(compressed_size));
    compressed_data.resize(compressed_size);
    return compressed_data;
}

// Compresses `data` into a zstd frame that has a checksum but does not record its content size.
Bytes compress_without_content_size(const Bytes &data) {
    return compress_with_zstd(data, {{ZSTD_c_contentSizeFlag, 0}, {ZSTD_c_checksumFlag, 1}});
}

// Compresses `data` into a zstd frame that records its content size but has no checksum.
Bytes compress_without_checksum(const Bytes &data) {
    return compress_with_zstd(data, {{ZSTD_c_contentSizeFlag, 1}, {ZSTD_c_checksumFlag, 0}});
}

} // namespace

TEST_CASE("compress_with_checksum computes CRC-32C as eight lowercase hexadecimal digits", "[io][compression]") {
    SECTION("check value") {
        const io::envelope::CompressedData compressed = REQUIRE_VALUE(io::envelope::compress_with_checksum(bytes_from_string("123456789"), CompressionAlgorithm::None, ChecksumAlgorithm::Crc32c));
        CHECK(compressed.checksum == "e3069283");
    }

    SECTION("empty input") {
        const io::envelope::CompressedData compressed = REQUIRE_VALUE(io::envelope::compress_with_checksum({}, CompressionAlgorithm::None, ChecksumAlgorithm::Crc32c));
        CHECK(compressed.checksum == "00000000");
    }
}

TEST_CASE("checked_decompress restores what compress_with_checksum compressed", "[io][compression]") {
    const auto [compression_algorithm, checksum_algorithm] = GENERATE(table<CompressionAlgorithm, ChecksumAlgorithm>({
        {CompressionAlgorithm::None, ChecksumAlgorithm::None},
        {CompressionAlgorithm::None, ChecksumAlgorithm::Crc32c},
        {CompressionAlgorithm::ZstdBestCompressionWithChecksum, ChecksumAlgorithm::HandledByCompressionLib},
        {CompressionAlgorithm::ZstdBestCompressionWithChecksum, ChecksumAlgorithm::Crc32c},
    }));
    const Bytes data = GENERATE(Bytes(4096, std::byte{0x2a}), Bytes{});

    const io::envelope::CompressedData compressed = REQUIRE_VALUE(io::envelope::compress_with_checksum(data, compression_algorithm, checksum_algorithm));
    const Bytes decompressed = REQUIRE_VALUE(io::envelope::checked_decompress(compressed.compressed_data, compression_algorithm, checksum_algorithm, compressed.checksum));

    CHECK(decompressed == data);
}

TEST_CASE("compress_with_checksum rejects unsupported algorithms and combinations", "[io][compression]") {
    const auto [compression_algorithm, checksum_algorithm, expected] = GENERATE(table<CompressionAlgorithm, ChecksumAlgorithm, CompressionError>({
        {CompressionAlgorithm::None, static_cast<ChecksumAlgorithm>(99), CompressionError::UnsupportedChecksumAlgorithm},
        {static_cast<CompressionAlgorithm>(99), ChecksumAlgorithm::None, CompressionError::UnsupportedCompressionAlgorithm},
        {CompressionAlgorithm::ZstdBestCompressionWithChecksum, ChecksumAlgorithm::None, CompressionError::InvalidAlgorithmCombination},
        {CompressionAlgorithm::None, ChecksumAlgorithm::HandledByCompressionLib, CompressionError::InvalidAlgorithmCombination},
    }));

    CHECK(REQUIRE_ERROR(io::envelope::compress_with_checksum(bytes_from_string("123456789"), compression_algorithm, checksum_algorithm)).holds(expected));
}

TEST_CASE("checked_decompress rejects unsupported algorithms and combinations", "[io][compression]") {
    const auto [compression_algorithm, checksum_algorithm, expected] = GENERATE(table<CompressionAlgorithm, ChecksumAlgorithm, CompressionError>({
        {CompressionAlgorithm::None, static_cast<ChecksumAlgorithm>(99), CompressionError::UnsupportedChecksumAlgorithm},
        {static_cast<CompressionAlgorithm>(99), ChecksumAlgorithm::None, CompressionError::UnsupportedCompressionAlgorithm},
        {CompressionAlgorithm::ZstdBestCompressionWithChecksum, ChecksumAlgorithm::None, CompressionError::InvalidAlgorithmCombination},
        {CompressionAlgorithm::None, ChecksumAlgorithm::HandledByCompressionLib, CompressionError::InvalidAlgorithmCombination},
    }));

    CHECK(REQUIRE_ERROR(io::envelope::checked_decompress(bytes_from_string("123456789"), compression_algorithm, checksum_algorithm, {})).holds(expected));
}

TEST_CASE("checked_decompress detects corruption", "[io][compression]") {
    const Bytes data(4096, std::byte{0x2a});

    SECTION("changed data without compression") {
        io::envelope::CompressedData compressed = REQUIRE_VALUE(io::envelope::compress_with_checksum(data, CompressionAlgorithm::None, ChecksumAlgorithm::Crc32c));
        compressed.compressed_data.front() ^= std::byte{0x01};

        CHECK(REQUIRE_ERROR(io::envelope::checked_decompress(compressed.compressed_data, CompressionAlgorithm::None, ChecksumAlgorithm::Crc32c, compressed.checksum)).holds(CompressionError::ChecksumMismatch));
    }

    SECTION("changed data with zstd") {
        io::envelope::CompressedData compressed = REQUIRE_VALUE(io::envelope::compress_with_checksum(data, CompressionAlgorithm::ZstdBestCompressionWithChecksum, ChecksumAlgorithm::HandledByCompressionLib));
        compressed.compressed_data.back() ^= std::byte{0x01};

        CHECK(REQUIRE_ERROR(io::envelope::checked_decompress(compressed.compressed_data, CompressionAlgorithm::ZstdBestCompressionWithChecksum, ChecksumAlgorithm::HandledByCompressionLib, {})).holds(CompressionError::ChecksumMismatch));
    }

    SECTION("wrong CRC-32C with zstd") {
        const io::envelope::CompressedData compressed = REQUIRE_VALUE(io::envelope::compress_with_checksum(data, CompressionAlgorithm::ZstdBestCompressionWithChecksum, ChecksumAlgorithm::Crc32c));

        CHECK(REQUIRE_ERROR(io::envelope::checked_decompress(compressed.compressed_data, CompressionAlgorithm::ZstdBestCompressionWithChecksum, ChecksumAlgorithm::Crc32c, "00000000")).holds(CompressionError::ChecksumMismatch));
    }

    SECTION("malformed CRC-32C") {
        const std::string_view checksum = GENERATE(std::string_view("E3069283"), std::string_view(""));

        CHECK(REQUIRE_ERROR(io::envelope::checked_decompress(bytes_from_string("123456789"), CompressionAlgorithm::None, ChecksumAlgorithm::Crc32c, checksum)).holds(CompressionError::ChecksumMismatch));
    }

    SECTION("zstd frame without a checksum") {
        const Bytes compressed_data = compress_without_checksum(data);

        CHECK(REQUIRE_ERROR(io::envelope::checked_decompress(compressed_data, CompressionAlgorithm::ZstdBestCompressionWithChecksum, ChecksumAlgorithm::HandledByCompressionLib, {})).holds(CompressionError::ChecksumMismatch));
    }
}

TEST_CASE("checked_decompress enforces its maximum size", "[io][compression]") {
    const Bytes data(4096, std::byte{0x2a});

    SECTION("zstd frame with its content size") {
        const Bytes compressed_data = REQUIRE_VALUE(io::envelope::compress_with_checksum(data, CompressionAlgorithm::ZstdBestCompressionWithChecksum, ChecksumAlgorithm::HandledByCompressionLib)).compressed_data;

        CHECK(REQUIRE_VALUE(io::envelope::checked_decompress(compressed_data, CompressionAlgorithm::ZstdBestCompressionWithChecksum, ChecksumAlgorithm::HandledByCompressionLib, {}, 4096)) == data);
        CHECK(REQUIRE_ERROR(io::envelope::checked_decompress(compressed_data, CompressionAlgorithm::ZstdBestCompressionWithChecksum, ChecksumAlgorithm::HandledByCompressionLib, {}, 4095)).holds(CompressionError::SizeLimitExceeded));
    }

    SECTION("zstd frame without its content size") {
        const Bytes compressed_data = compress_without_content_size(data);

        CHECK(REQUIRE_VALUE(io::envelope::checked_decompress(compressed_data, CompressionAlgorithm::ZstdBestCompressionWithChecksum, ChecksumAlgorithm::HandledByCompressionLib, {}, 4096)) == data);
        CHECK(REQUIRE_ERROR(io::envelope::checked_decompress(compressed_data, CompressionAlgorithm::ZstdBestCompressionWithChecksum, ChecksumAlgorithm::HandledByCompressionLib, {}, 4095)).holds(CompressionError::SizeLimitExceeded));
    }

    SECTION("no compression") {
        CHECK(REQUIRE_VALUE(io::envelope::checked_decompress(data, CompressionAlgorithm::None, ChecksumAlgorithm::None, {}, 4096)) == data);
        CHECK(REQUIRE_ERROR(io::envelope::checked_decompress(data, CompressionAlgorithm::None, ChecksumAlgorithm::None, {}, 4095)).holds(CompressionError::SizeLimitExceeded));
    }
}
