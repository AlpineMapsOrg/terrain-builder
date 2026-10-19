#pragma once

#include <zpp_bits.h>

#include <limits>

#include <fmt/format.h>

#include "error/try.h"
#include "error/utils.h"
#include "type_utils.h"

namespace io::envelope {
namespace detail {

template <typename Value>
alp::Result<Bytes, SerdeError> serialize_to_bytes(const Value &value) {
    Bytes bytes;
    zpp::bits::out output(bytes, zpp::bits::alloc_limit<default_max_decompressed_size>());
    const zpp::bits::errc result = output(value);
    if (zpp::bits::failure(result)) {
        return alp::make_unexpected(SerdeError(result.code), fmt::format("encoding {}", type_name<Value>()));
    }
    return bytes;
}

template <typename Value>
alp::Result<Value, EnvelopeError, SerdeError> deserialize_from_bytes(const std::span<const std::byte> bytes) {
    Value value{};
    zpp::bits::in input(bytes, zpp::bits::alloc_limit<default_max_decompressed_size>());
    const zpp::bits::errc result = input(value);
    if (zpp::bits::failure(result)) {
        return alp::make_unexpected(SerdeError(result.code), fmt::format("decoding {}", type_name<Value>()));
    }
    if (input.position() != bytes.size()) {
        return alp::make_unexpected(EnvelopeError::TrailingData, fmt::format("decoding {}", type_name<Value>()));
    }
    return value;
}

template <typename Schema, std::size_t Index, typename Current>
typename Schema::Latest convert_to_latest(Current current) {
    if constexpr (Index + 1 == Schema::version_count) {
        return current;
    } else {
        using Next = typename Schema::template VersionAt<Index + 1>::Payload;
        return convert_to_latest<Schema, Index + 1>(Next::from_previous(std::move(current)));
    }
}

template <typename Schema, std::size_t Index = 0>
alp::Result<typename Schema::Latest, EnvelopeError, SerdeError> deserialize_version(
    const std::uint32_t class_version,
    const std::span<const std::byte> payload_bytes) {
    using CurrentVersion = typename Schema::template VersionAt<Index>;
    if (class_version == CurrentVersion::number) {
        ALP_TRY(auto payload, deserialize_from_bytes<typename CurrentVersion::Payload>(payload_bytes));
        return convert_to_latest<Schema, Index>(std::move(payload));
    }

    if constexpr (Index + 1 < Schema::version_count) {
        return deserialize_version<Schema, Index + 1>(class_version, payload_bytes);
    } else {
        return alp::make_unexpected(EnvelopeError::UnsupportedClassVersion);
    }
}

} // namespace detail

template <typename Schema, std::uint32_t VersionNumber>
alp::Result<Bytes, CompressionError, SerdeError> serialize(
    const typename Schema::template Payload<VersionNumber> &payload,
    const CompressionAlgorithm compression_algorithm,
    const ChecksumAlgorithm checksum_algorithm) {
    static_assert(Schema::template supports_version<VersionNumber>,
                  "the requested payload version is not part of the schema");
    const auto context = alp::make_context("serializing {} version {}", Schema::class_name, VersionNumber);

    ALP_TRY_CONTEXT(const Bytes payload_bytes, detail::serialize_to_bytes(payload), context);
    ALP_TRY_CONTEXT(CompressedData compressed, compress_with_checksum(payload_bytes, compression_algorithm, checksum_algorithm), context);

    const Envelope envelope{
        .magic = magic,
        .class_name = std::string{Schema::class_name},
        .class_version = VersionNumber,
        .checksum_algorithm = checksum_algorithm,
        .checksum = std::move(compressed.checksum),
        .compression_algorithm = compression_algorithm,
        .uncompressed_size = payload_bytes.size(),
        .compressed_data = std::move(compressed.compressed_data),
    };
    ALP_TRY_CONTEXT(Bytes bytes, detail::serialize_to_bytes(envelope), context);
    return bytes;
}

template <typename Schema>
alp::Result<typename Schema::Latest, EnvelopeError, CompressionError, SerdeError> deserialize(
    const std::span<const std::byte> bytes,
    const std::size_t max_decompressed_size) {
    const auto context = alp::make_context("deserializing {}", Schema::class_name);

    ALP_TRY_CONTEXT(const Envelope envelope, detail::deserialize_from_bytes<Envelope>(bytes), context);
    if (envelope.magic != magic) {
        return alp::make_unexpected(EnvelopeError::InvalidMagic, context);
    }
    if (envelope.class_name != Schema::class_name) {
        return alp::make_unexpected(EnvelopeError::WrongClassName, context);
    }
    if (!Schema::supports_version_number(envelope.class_version)) {
        return alp::make_unexpected(EnvelopeError::UnsupportedClassVersion, context);
    }

    const std::size_t effective_max_decompressed_size =
        std::min(max_decompressed_size, default_max_decompressed_size);
    if (envelope.uncompressed_size > std::numeric_limits<std::size_t>::max()
        || envelope.uncompressed_size > effective_max_decompressed_size) {
        return alp::make_unexpected(EnvelopeError::SizeLimitExceeded, context);
    }

    auto payload_bytes_result = checked_decompress(
        envelope.compressed_data,
        envelope.compression_algorithm,
        envelope.checksum_algorithm,
        envelope.checksum,
        static_cast<std::size_t>(envelope.uncompressed_size));
    // The limit is the declared size, so a payload beyond it is larger than declared.
    if (!payload_bytes_result && payload_bytes_result.error().holds(CompressionError::SizeLimitExceeded)) {
        return alp::make_unexpected(EnvelopeError::UncompressedSizeMismatch, context);
    }
    ALP_TRY_CONTEXT(const Bytes payload_bytes, std::move(payload_bytes_result), context);
    if (payload_bytes.size() != envelope.uncompressed_size) {
        return alp::make_unexpected(EnvelopeError::UncompressedSizeMismatch, context);
    }

    ALP_TRY_CONTEXT(auto latest, detail::deserialize_version<Schema>(envelope.class_version, payload_bytes), context);
    return latest;
}

} // namespace io::envelope
