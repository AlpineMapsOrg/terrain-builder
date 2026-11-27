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

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>

#include <libassert/assert.hpp>

#include "common_error_codes.h"
#include "error/Error.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "fmt_impls.h"
#include "io/PayloadSchema.h"
#include "io/WriteMode.h"
#include "io/bytes.h"
#include "io/compression.h"
#include "io/glm_serialization.h"
#include "io/hash.h"
#include "io/limits.h"
#include "io/serde.h"

namespace io::envelope {

inline constexpr uint64_t magic = 0xA6EFA707D12E4404ULL;
inline constexpr size_t max_class_name_size = 128;
inline constexpr size_t max_hash_size = 64;

struct Header {
    uint64_t magic;
    std::string class_name;
    uint32_t class_version;
    hash::Algorithm hash_algorithm;
    // hash::compute of the uncompressed payload.
    bytes::vector hash;
    compression::Algorithm compression_algorithm;
    uint64_t uncompressed_size;
};

// The payload is the last field, so a file prefix of max_header_size bytes contains the header.
struct Envelope {
    Header header;
    bytes::vector compressed_data;
};

// zpp_bits prefixes strings and vectors with their 32-bit size.
inline constexpr size_t max_header_size = sizeof(uint64_t) + sizeof(uint32_t) + max_class_name_size + sizeof(uint32_t)
    + sizeof(hash::Algorithm) + sizeof(uint32_t) + max_hash_size + sizeof(compression::Algorithm) + sizeof(uint64_t);

struct CorruptData : alp::MessageCode<"the data is not a valid envelope"> {};

using HeaderError = alp::Error<CorruptData, Unsupported>;

// How `serialize` encodes the payload.
struct SerializeOptions {
    compression::Algorithm compression_algorithm = compression::Algorithm::ZstdDefault;
    hash::Algorithm hash_algorithm = hash::Algorithm::Xxh3_64;
};

using SerializeError = alp::Error<serde::SerializeError, compression::CompressError>;
// Encodes `payload` as version `VersionNumber` of `Schema`.
template <typename Schema, uint32_t VersionNumber>
[[nodiscard]]
alp::Result<bytes::vector, SerializeError> serialize(const typename Schema::template Payload<VersionNumber> &payload, SerializeOptions options = {});

// Encodes `payload` as the latest version of `Schema`.
template <typename Schema>
[[nodiscard]]
alp::Result<bytes::vector, SerializeError> serialize(const typename Schema::Latest &payload, SerializeOptions options = {});

// What `deserialize` accepts.
struct DeserializeOptions {
    // The hash the header must record, if given.
    std::optional<bytes::span> expected_hash = std::nullopt;
    // The largest payload, at most `max_buffer_size`.
    size_t max_decompressed_size = max_buffer_size;
};

using DeserializeError = alp::Error<HeaderError, BufferLimitExceeded, compression::DecompressError>;
// Decodes an envelope of `Schema` and upgrades its payload to the latest version.
template <typename Schema>
[[nodiscard]]
alp::Result<typename Schema::Latest, DeserializeError> deserialize(bytes::span bytes, DeserializeOptions options = {});

// What `read` accepts.
using ReadOptions = DeserializeOptions;

using ReadError = alp::Error<bytes::ReadError, DeserializeError>;
// Reads the envelope of `Schema` in the file at `path`.
template <typename Schema>
[[nodiscard]]
alp::Result<typename Schema::Latest, ReadError> read(const std::filesystem::path &path, ReadOptions options = {});

// How `write` encodes the payload and treats an existing file and missing parent directories.
struct WriteOptions {
    WriteMode mode = default_write_mode;
    bool make_dirs = default_make_dirs;
    compression::Algorithm compression_algorithm = compression::Algorithm::ZstdDefault;
    hash::Algorithm hash_algorithm = hash::Algorithm::Xxh3_64;
};

using WriteError = alp::Error<SerializeError, bytes::WriteError>;
// Writes `payload` as an envelope of `Schema` to the file at `path`, and returns the hash its header records.
template <typename Schema>
[[nodiscard]]
alp::Result<bytes::vector, WriteError> write(const typename Schema::Latest &payload, const std::filesystem::path &path, WriteOptions options = {});

using ReadHeaderError = alp::Error<bytes::ReadError, HeaderError>;
// Reads the header of the envelope in the file at `path`, without its payload.
// Checks neither its class nor its hash.
[[nodiscard]]
alp::Result<Header, ReadHeaderError> read_header(const std::filesystem::path &path);

// An error from reading or writing an envelope.
using Error = alp::Error<ReadError, WriteError>;

namespace detail {

// Checks the magic number, the field limits and the algorithms of `header`.
[[nodiscard]]
alp::Result<void, HeaderError> validate_header(const Header &header);

// The envelope's error for a failure decoding its data: running out of memory passes through, any other failure means the data is not a valid envelope.
[[nodiscard]]
alp::Error<CorruptData, OutOfMemory> to_envelope_error(serde::DeserializeError error, alp::Context auto &&context) {
    auto split_result = std::move(error).split<OutOfMemory>();
    if (split_result.has_value()) {
        return std::move(split_result).value().context(std::forward<decltype(context)>(context));
    }
    return alp::make_error(CorruptData{}, std::forward<decltype(context)>(context), std::move(split_result).error());
}

template <typename Schema, size_t Index, typename Current>
typename Schema::Latest convert_to_latest(Current current) {
    if constexpr (Index + 1 == Schema::version_count) {
        return current;
    } else {
        using Next = typename Schema::template VersionAt<Index + 1>::Payload;
        return convert_to_latest<Schema, Index + 1>(Next::from_previous(std::move(current)));
    }
}

// Decodes a payload of version `class_version`, which the schema must have, and upgrades it to the latest version.
template <typename Schema, size_t Index = 0>
alp::Result<typename Schema::Latest, serde::DeserializeError> deserialize_version(const uint32_t class_version, const bytes::span payload_bytes) {
    using CurrentVersion = typename Schema::template VersionAt<Index>;
    if (class_version == CurrentVersion::number) {
        ALP_TRY(auto payload, serde::deserialize<typename CurrentVersion::Payload>(payload_bytes));
        return convert_to_latest<Schema, Index>(std::move(payload));
    }

    if constexpr (Index + 1 < Schema::version_count) {
        return deserialize_version<Schema, Index + 1>(class_version, payload_bytes);
    } else {
        UNREACHABLE("the schema has no payload version", class_version);
    }
}

template <typename Schema, uint32_t VersionNumber>
alp::Result<Envelope, SerializeError> make_envelope(const typename Schema::template Payload<VersionNumber> &payload, const SerializeOptions options) {
    static_assert(Schema::template supports_version<VersionNumber>, "the requested payload version is not part of the schema");
    static_assert(Schema::class_name.size() <= max_class_name_size, "the class name exceeds the envelope limit");

    ALP_TRY(bytes::vector payload_bytes, serde::serialize(payload));
    ALP_TRY(bytes::vector compressed_data, compression::compress(payload_bytes, options.compression_algorithm));

    Envelope envelope{
        .header = {
            .magic = magic,
            .class_name = std::string{Schema::class_name},
            .class_version = VersionNumber,
            .hash_algorithm = options.hash_algorithm,
            .hash = hash::compute(payload_bytes, options.hash_algorithm),
            .compression_algorithm = options.compression_algorithm,
            .uncompressed_size = payload_bytes.size(),
        },
        .compressed_data = std::move(compressed_data),
    };
    ASSERT(envelope.header.hash.size() <= max_hash_size);
    return envelope;
}

} // namespace detail

template <typename Schema, uint32_t VersionNumber>
alp::Result<bytes::vector, SerializeError> serialize(const typename Schema::template Payload<VersionNumber> &payload, const SerializeOptions options) {
    const auto context = alp::make_context("encoding {} version {}", Schema::class_name, VersionNumber);
    ALP_TRY_CONTEXT(const Envelope envelope, (detail::make_envelope<Schema, VersionNumber>(payload, options)), context);
    ALP_TRY_CONTEXT(bytes::vector bytes, serde::serialize(envelope), context);
    return bytes;
}

template <typename Schema>
alp::Result<bytes::vector, SerializeError> serialize(const typename Schema::Latest &payload, const SerializeOptions options) {
    return serialize<Schema, Schema::latest_version>(payload, options);
}

template <typename Schema>
alp::Result<typename Schema::Latest, DeserializeError> deserialize(const bytes::span bytes, const DeserializeOptions options) {
    const auto context = alp::make_context("decoding {}", Schema::class_name);

    auto envelope_result = serde::deserialize<Envelope>(bytes);
    if (!envelope_result.has_value()) {
        return detail::to_envelope_error(std::move(envelope_result).error(), context);
    }
    const Envelope &envelope = envelope_result.value();
    const Header &header = envelope.header;
    ALP_TRY_CONTEXT(detail::validate_header(header), context);
    if (header.class_name != Schema::class_name) {
        return alp::make_error(CorruptData{}, fmt::format("the envelope holds the class \"{}\"", header.class_name)).context(context);
    }
    if (!Schema::supports_version_number(header.class_version)) {
        return alp::make_error(Unsupported{}, fmt::format("the envelope holds the class version {}", header.class_version)).context(context);
    }

    const size_t max_decompressed_size = std::min(options.max_decompressed_size, max_buffer_size);
    if (header.uncompressed_size > max_decompressed_size) {
        return alp::make_error(BufferLimitExceeded(max_decompressed_size, header.uncompressed_size), context);
    }

    auto payload_bytes_result = compression::decompress(envelope.compressed_data, header.compression_algorithm, header.uncompressed_size);
    // The limit is the declared size, so a payload beyond it is larger than declared.
    if (!payload_bytes_result.has_value() && payload_bytes_result.error().holds<BufferLimitExceeded>()) {
        return alp::make_error(CorruptData{}, fmt::format("the payload is larger than the {} bytes the header declares", header.uncompressed_size), std::move(payload_bytes_result).error()).context(context);
    }
    ALP_TRY_CONTEXT(const bytes::vector payload_bytes, std::move(payload_bytes_result), context);
    if (payload_bytes.size() != header.uncompressed_size) {
        return alp::make_error(CorruptData{}, fmt::format("the payload has {} bytes, not the {} bytes the header declares", payload_bytes.size(), header.uncompressed_size)).context(context);
    }
    if (hash::compute(payload_bytes, header.hash_algorithm) != header.hash) {
        return alp::make_error(CorruptData{}, "the payload does not have the hash the header records").context(context);
    }
    if (options.expected_hash.has_value() && !std::ranges::equal(header.hash, *options.expected_hash)) {
        return alp::make_error(CorruptData{}, "the header records a different hash than expected").context(context);
    }

    auto latest_result = detail::deserialize_version<Schema>(header.class_version, payload_bytes);
    if (!latest_result.has_value()) {
        return detail::to_envelope_error(std::move(latest_result).error(), "decoding the payload").context(context);
    }
    return std::move(latest_result).value();
}

template <typename Schema>
alp::Result<typename Schema::Latest, ReadError> read(const std::filesystem::path &path, const ReadOptions options) {
    const auto context = alp::make_context("reading {} from {}", Schema::class_name, path);
    ALP_TRY_CONTEXT(const bytes::vector bytes, bytes::read(path), context);
    ALP_TRY_CONTEXT(typename Schema::Latest latest, deserialize<Schema>(bytes, options), context);
    return latest;
}

template <typename Schema>
alp::Result<bytes::vector, WriteError> write(const typename Schema::Latest &payload, const std::filesystem::path &path, const WriteOptions options) {
    const auto context = alp::make_context("writing {} to {}", Schema::class_name, path);
    const SerializeOptions serialize_options{.compression_algorithm = options.compression_algorithm, .hash_algorithm = options.hash_algorithm};
    ALP_TRY_CONTEXT(Envelope envelope, (detail::make_envelope<Schema, Schema::latest_version>(payload, serialize_options)), context);
    ALP_TRY_CONTEXT(const bytes::vector bytes, serde::serialize(envelope), context);
    ALP_TRY_CONTEXT(bytes::write(bytes, path, {.mode = options.mode, .make_dirs = options.make_dirs}), context);
    return std::move(envelope.header.hash);
}

} // namespace io::envelope
