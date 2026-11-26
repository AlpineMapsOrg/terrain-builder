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

#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"

#include "common_error_codes.h"
#include "io/FileError.h"
#include "io/bytes.h"
#include "io/envelope.h"

#include <zpp_bits.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace v1 {

    struct Payload {
        uint32_t id;
        std::string name;

        bool operator==(const Payload&) const = default;
    };

} // namespace v1

namespace v2 {

    struct Payload {
        uint64_t id;
        std::string name;
        bool enabled;

        static Payload from_previous(v1::Payload previous)
        {
            return {
                .id = previous.id,
                .name = std::move(previous.name),
                .enabled = true,
            };
        }

        bool operator==(const Payload&) const = default;
    };

} // namespace v2

namespace v3 {

    struct Payload {
        uint64_t id;
        std::string label;
        bool enabled;
        std::vector<int32_t> samples;

        static Payload from_previous(v2::Payload previous)
        {
            return {
                .id = previous.id,
                .label = std::move(previous.name),
                .enabled = previous.enabled,
                .samples = {},
            };
        }

        bool operator==(const Payload&) const = default;
    };

} // namespace v3

using Schema = io::envelope::
    PayloadSchema<"test.Payload", io::envelope::Version<1, v1::Payload>, io::envelope::Version<2, v2::Payload>, io::envelope::Version<3, v3::Payload>>;

constexpr std::array zstd_compression_algorithms {
    io::compression::Algorithm::ZstdBest,
    io::compression::Algorithm::ZstdDefault,
};

constexpr std::array compression_algorithms {
    io::compression::Algorithm::None,
    io::compression::Algorithm::ZstdBest,
    io::compression::Algorithm::ZstdDefault,
};

constexpr std::array hash_algorithms {
    io::hash::Algorithm::None,
    io::hash::Algorithm::Xxh3_64,
};

static_assert(std::is_aggregate_v<v1::Payload>);
static_assert(std::is_aggregate_v<v2::Payload>);
static_assert(std::is_aggregate_v<v3::Payload>);
static_assert(Schema::class_name == "test.Payload");
static_assert(Schema::latest_version == 3);
static_assert(std::same_as<Schema::Latest, v3::Payload>);
static_assert(std::same_as<Schema::Payload<1>, v1::Payload>);

template <typename Value>
io::bytes::vector encode_value(const Value& value)
{
    io::bytes::vector bytes;
    zpp::bits::out output(bytes);
    output(value).or_throw();
    return bytes;
}

io::bytes::vector encode_envelope(const io::envelope::Envelope& envelope) { return encode_value(envelope); }

io::envelope::Envelope decode_envelope(const io::bytes::vector& bytes)
{
    io::envelope::Envelope envelope {};
    zpp::bits::in input(bytes);
    input(envelope).or_throw();
    return envelope;
}

void write_file(const std::filesystem::path& path, const io::bytes::span bytes) { REQUIRE(io::bytes::write(bytes, path, {.mode = io::WriteMode::Overwrite})); }

} // namespace

TEST_CASE("Envelope round trips the latest payload version", "[io][envelope]")
{
    const v3::Payload expected {
        .id = 42,
        .label = "latest",
        .enabled = false,
        .samples = { 1, 2, 3 },
    };

    const auto bytes = io::envelope::serialize<Schema, 3>(expected);
    REQUIRE(bytes.has_value());

    const auto envelope = decode_envelope(*bytes);
    CHECK(envelope.header.magic == io::envelope::magic);
    CHECK(envelope.header.class_name == Schema::class_name);
    CHECK(envelope.header.class_version == 3);
    CHECK(envelope.header.hash_algorithm == io::hash::Algorithm::Xxh3_64);
    CHECK(envelope.header.hash == io::hash::data(encode_value(expected), io::hash::Algorithm::Xxh3_64));
    CHECK(envelope.header.compression_algorithm == io::compression::Algorithm::ZstdDefault);
    CHECK(envelope.header.uncompressed_size == encode_value(expected).size());

    const auto result = io::envelope::deserialize<Schema>(*bytes);
    REQUIRE(result.has_value());
    CHECK(*result == expected);
}

TEST_CASE("Envelope round trips payloads containing GLM types", "[io][envelope]")
{
    struct Payload {
        glm::dvec3 position;
        glm::mat4 transform;
        std::vector<glm::vec2> texture_coordinates;
        bool operator==(const Payload&) const = default;
    };
    using GlmSchema = io::envelope::PayloadSchema<"test.GlmPayload", io::envelope::Version<1, Payload>>;
    const Payload expected {
        .position = { 1.0, -2.0, 3.5 },
        .transform = glm::mat4(2.0f),
        .texture_coordinates = { { 0.0f, 0.5f }, { 1.0f, 0.75f } },
    };
    const auto bytes = io::envelope::serialize<GlmSchema>(expected);
    REQUIRE(bytes.has_value());
    const auto result = io::envelope::deserialize<GlmSchema>(*bytes);
    REQUIRE(result.has_value());
    CHECK(*result == expected);
}

TEST_CASE("Envelope upgrades older payload versions", "[io][envelope]")
{
    SECTION("version 1 is upgraded through every subsequent version")
    {
        const v1::Payload original { .id = 7, .name = "version one" };
        const auto bytes = io::envelope::serialize<Schema, 1>(original);
        REQUIRE(bytes.has_value());

        const auto result = io::envelope::deserialize<Schema>(*bytes);
        REQUIRE(result.has_value());
        CHECK(*result
            == v3::Payload {
                .id = 7,
                .label = "version one",
                .enabled = true,
                .samples = {},
            });
    }

    SECTION("version 2 is upgraded to the latest version")
    {
        const v2::Payload original { .id = 9, .name = "version two", .enabled = false };
        const auto bytes = io::envelope::serialize<Schema, 2>(original);
        REQUIRE(bytes.has_value());

        const auto result = io::envelope::deserialize<Schema>(*bytes);
        REQUIRE(result.has_value());
        CHECK(*result
            == v3::Payload {
                .id = 9,
                .label = "version two",
                .enabled = false,
                .samples = {},
            });
    }
}

TEST_CASE("Envelope supports uncompressed data without a hash", "[io][envelope]")
{
    const v3::Payload expected {
        .id = 11,
        .label = "plain",
        .enabled = true,
        .samples = { 5, 8 },
    };
    const auto bytes = io::envelope::serialize<Schema, 3>(expected, {.compression_algorithm = io::compression::Algorithm::None, .hash_algorithm = io::hash::Algorithm::None});
    REQUIRE(bytes.has_value());

    const auto envelope = decode_envelope(*bytes);
    CHECK(envelope.header.compression_algorithm == io::compression::Algorithm::None);
    CHECK(envelope.header.hash_algorithm == io::hash::Algorithm::None);
    CHECK(envelope.header.hash.empty());
    CHECK(envelope.header.uncompressed_size == envelope.compressed_data.size());

    const auto result = io::envelope::deserialize<Schema>(*bytes);
    REQUIRE(result.has_value());
    CHECK(*result == expected);
}

TEST_CASE("Envelope round trips every compression and hash combination", "[io][envelope]")
{
    const v3::Payload expected {
        .id = 12,
        .label = "combination",
        .enabled = true,
        .samples = { 3, 5, 8 },
    };
    for (const auto compression_algorithm : compression_algorithms) {
        for (const auto hash_algorithm : hash_algorithms) {
            CAPTURE(compression_algorithm, hash_algorithm);
            const auto bytes = io::envelope::serialize<Schema, 3>(expected, {.compression_algorithm = compression_algorithm, .hash_algorithm = hash_algorithm});
            REQUIRE(bytes.has_value());

            const auto envelope = decode_envelope(*bytes);
            CHECK(envelope.header.compression_algorithm == compression_algorithm);
            CHECK(envelope.header.hash_algorithm == hash_algorithm);
            CHECK(envelope.header.hash == io::hash::data(encode_value(expected), hash_algorithm));

            const auto result = io::envelope::deserialize<Schema>(*bytes);
            REQUIRE(result.has_value());
            CHECK(*result == expected);
        }
    }
}

TEST_CASE("XXH3-64 detects corrupted envelope payloads", "[io][envelope]")
{
    const v3::Payload payload { .id = 15, .label = "protected by XXH3-64", .enabled = true, .samples = { 13, 21 } };

    SECTION("without compression")
    {
        const auto bytes = io::envelope::serialize<Schema, 3>(payload, {.compression_algorithm = io::compression::Algorithm::None});
        REQUIRE(bytes.has_value());
        auto envelope = decode_envelope(*bytes);
        envelope.compressed_data.back() ^= uint8_t { 0x01 };
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<io::envelope::CorruptData>());
    }

    SECTION("with zstd compression")
    {
        for (const auto compression_algorithm : zstd_compression_algorithms) {
            CAPTURE(compression_algorithm);
            const auto bytes = io::envelope::serialize<Schema, 3>(payload, {.compression_algorithm = compression_algorithm});
            REQUIRE(bytes.has_value());
            auto envelope = decode_envelope(*bytes);
            envelope.compressed_data.back() ^= uint8_t { 0x01 };
            CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<io::envelope::CorruptData>());
        }
    }

    SECTION("older version")
    {
        const v1::Payload original { .id = 16, .name = "xxh3 version one" };
        const auto bytes = io::envelope::serialize<Schema, 1>(original, {.compression_algorithm = io::compression::Algorithm::None});
        REQUIRE(bytes.has_value());
        auto envelope = decode_envelope(*bytes);
        REQUIRE(io::envelope::deserialize<Schema>(encode_envelope(envelope)).has_value());
        envelope.compressed_data.front() ^= uint8_t { 0x01 };
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<io::envelope::CorruptData>());
    }
}

TEST_CASE("Envelope rejects incompatible metadata", "[io][envelope]")
{
    const v3::Payload payload { .id = 1, .label = "metadata", .enabled = true, .samples = {} };
    const auto serialized = io::envelope::serialize<Schema, 3>(payload);
    REQUIRE(serialized.has_value());

    SECTION("magic")
    {
        auto envelope = decode_envelope(*serialized);
        ++envelope.header.magic;
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<io::envelope::CorruptData>());
    }

    SECTION("magic of the previous format")
    {
        auto envelope = decode_envelope(*serialized);
        envelope.header.magic = 0xF5FBD3EF919428CAULL;
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<io::envelope::CorruptData>());
    }

    SECTION("class name")
    {
        auto envelope = decode_envelope(*serialized);
        envelope.header.class_name = "other.Payload";
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<io::envelope::CorruptData>());
    }

    SECTION("class name exceeds its limit")
    {
        auto envelope = decode_envelope(*serialized);
        envelope.header.class_name = std::string(io::envelope::max_class_name_size + 1, 'x');
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<io::envelope::CorruptData>());
    }

    SECTION("class version")
    {
        auto envelope = decode_envelope(*serialized);
        envelope.header.class_version = 99;
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<Unsupported>());
    }

    SECTION("hash algorithm")
    {
        auto envelope = decode_envelope(*serialized);
        envelope.header.hash_algorithm = static_cast<io::hash::Algorithm>(99);
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<Unsupported>());
    }

    SECTION("compression algorithm")
    {
        auto envelope = decode_envelope(*serialized);
        envelope.header.compression_algorithm = static_cast<io::compression::Algorithm>(99);
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<Unsupported>());
    }

    SECTION("hash for the None algorithm")
    {
        auto envelope = decode_envelope(*serialized);
        envelope.header.hash_algorithm = io::hash::Algorithm::None;
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<io::envelope::CorruptData>());
    }

    SECTION("incorrect hash")
    {
        auto envelope = decode_envelope(*serialized);
        envelope.header.hash.front() ^= uint8_t { 0x01 };
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<io::envelope::CorruptData>());
    }

    SECTION("hash with the wrong length")
    {
        auto envelope = decode_envelope(*serialized);
        envelope.header.hash.pop_back();
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<io::envelope::CorruptData>());
    }

    SECTION("hash exceeds its limit")
    {
        auto envelope = decode_envelope(*serialized);
        envelope.header.hash.resize(io::envelope::max_hash_size + 1);
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<io::envelope::CorruptData>());
    }

    SECTION("uncompressed size is smaller than the payload")
    {
        auto envelope = decode_envelope(*serialized);
        --envelope.header.uncompressed_size;
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<io::envelope::CorruptData>());
    }

    SECTION("uncompressed size is larger than the payload")
    {
        auto envelope = decode_envelope(*serialized);
        ++envelope.header.uncompressed_size;
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<io::envelope::CorruptData>());
    }

    SECTION("zero uncompressed size does not mean unspecified")
    {
        auto envelope = decode_envelope(*serialized);
        envelope.header.uncompressed_size = 0;
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<io::envelope::CorruptData>());
    }

    SECTION("uncompressed size exceeds the hard limit")
    {
        auto envelope = decode_envelope(*serialized);
        envelope.header.uncompressed_size = io::max_buffer_size + 1;
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<io::BufferLimitExceeded>());
    }

    SECTION("uncompressed size exceeds a caller limit")
    {
        auto envelope = decode_envelope(*serialized);
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope), {.max_decompressed_size = static_cast<size_t>(envelope.header.uncompressed_size - 1)})).holds<io::BufferLimitExceeded>());
    }
}

TEST_CASE("Envelope validates the size of uncompressed data", "[io][envelope]")
{
    const v3::Payload payload { .id = 2, .label = "plain", .enabled = true, .samples = { 1 } };
    const auto serialized = io::envelope::serialize<Schema, 3>(payload, {.compression_algorithm = io::compression::Algorithm::None, .hash_algorithm = io::hash::Algorithm::None});
    REQUIRE(serialized.has_value());

    SECTION("declared size is smaller")
    {
        auto envelope = decode_envelope(*serialized);
        --envelope.header.uncompressed_size;
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<io::envelope::CorruptData>());
    }

    SECTION("declared size is larger")
    {
        auto envelope = decode_envelope(*serialized);
        ++envelope.header.uncompressed_size;
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<io::envelope::CorruptData>());
    }
}

TEST_CASE("Envelope reports malformed serialized data", "[io][envelope]")
{
    SECTION("envelope")
    {
        const io::bytes::vector malformed { uint8_t { 0x01 }, uint8_t { 0x02 } };
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(malformed)).holds<io::envelope::CorruptData>());
    }

    SECTION("trailing bytes")
    {
        const v3::Payload payload { .id = 3, .label = "trailing", .enabled = true, .samples = {} };
        auto serialized = io::envelope::serialize<Schema, 3>(payload);
        REQUIRE(serialized.has_value());
        serialized->push_back(uint8_t { 0 });
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(*serialized)).holds<io::envelope::CorruptData>());
    }

    SECTION("payload")
    {
        const io::envelope::Envelope envelope {
            .header = {
                .magic = io::envelope::magic,
                .class_name = std::string { Schema::class_name },
                .class_version = 3,
                .hash_algorithm = io::hash::Algorithm::None,
                .hash = {},
                .compression_algorithm = io::compression::Algorithm::None,
                .uncompressed_size = 1,
            },
            .compressed_data = { uint8_t { 0x01 } },
        };
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_envelope(envelope))).holds<io::envelope::CorruptData>());
    }
}

TEST_CASE("Envelope headers are read without the payload", "[io][envelope]")
{
    TempDirectory directory("envelope-header");
    const auto path = directory.path() / "payload.envelope";

    SECTION("the header matches a full read")
    {
        const v3::Payload payload { .id = 21, .label = std::string(4096, 'h'), .enabled = true, .samples = { 1, 2, 3 } };
        REQUIRE(io::envelope::write<Schema>(payload, path, {.mode = io::WriteMode::Overwrite}));
        const auto header = io::envelope::read_header(path);
        REQUIRE(header.has_value());
        CHECK(header->magic == io::envelope::magic);
        CHECK(header->class_name == Schema::class_name);
        CHECK(header->class_version == Schema::latest_version);
        CHECK(header->hash_algorithm == io::hash::Algorithm::Xxh3_64);
        CHECK(header->compression_algorithm == io::compression::Algorithm::ZstdDefault);

        const auto full = io::envelope::read<Schema>(path);
        REQUIRE(full.has_value());
        CHECK(header->uncompressed_size == encode_value(*full).size());
        CHECK(header->hash == io::hash::data(encode_value(*full), io::hash::Algorithm::Xxh3_64));
    }

    SECTION("the class name is not checked")
    {
        io::envelope::Envelope envelope = decode_envelope(io::envelope::serialize<Schema, 3>({}).value());
        envelope.header.class_name = "other.Payload";
        write_file(path, encode_envelope(envelope));
        const auto header = io::envelope::read_header(path);
        REQUIRE(header.has_value());
        CHECK(header->class_name == "other.Payload");
    }

    SECTION("a header of the maximum size")
    {
        io::envelope::Envelope envelope = decode_envelope(io::envelope::serialize<Schema, 3>({}).value());
        envelope.header.class_name = std::string(io::envelope::max_class_name_size, 'n');
        envelope.header.hash = io::bytes::vector(io::envelope::max_hash_size, uint8_t { 0x5a });
        envelope.compressed_data = io::bytes::vector(1024, uint8_t { 0x11 });
        const auto bytes = encode_envelope(envelope);
        CHECK(encode_value(envelope.header).size() == io::envelope::max_header_size);
        write_file(path, bytes);
        const auto header = io::envelope::read_header(path);
        REQUIRE(header.has_value());
        CHECK(header->class_name == envelope.header.class_name);
        CHECK(header->hash == envelope.header.hash);
    }

    SECTION("a file shorter than the maximum header")
    {
        const io::envelope::Envelope envelope {
            .header = {
                .magic = io::envelope::magic,
                .class_name = "a",
                .class_version = 1,
                .hash_algorithm = io::hash::Algorithm::None,
                .hash = {},
                .compression_algorithm = io::compression::Algorithm::None,
                .uncompressed_size = 0,
            },
            .compressed_data = {},
        };
        const auto bytes = encode_envelope(envelope);
        REQUIRE(bytes.size() < io::envelope::max_header_size);
        write_file(path, bytes);
        const auto header = io::envelope::read_header(path);
        REQUIRE(header.has_value());
        CHECK(header->class_name == "a");
        CHECK(header->hash.empty());
    }

    SECTION("a truncated header")
    {
        const auto bytes = io::envelope::serialize<Schema, 3>({}).value();
        write_file(path, std::span(bytes).first(20));
        CHECK(REQUIRE_ERROR(io::envelope::read_header(path)).holds<io::envelope::CorruptData>());
    }

    SECTION("a corrupt class name length is rejected")
    {
        auto bytes = io::envelope::serialize<Schema, 3>({}).value();
        // The class name length follows the 8-byte magic.
        std::fill_n(bytes.begin() + 8, 4, uint8_t { 0xff });
        write_file(path, bytes);
        CHECK(REQUIRE_ERROR(io::envelope::read_header(path)).holds<io::envelope::CorruptData>());
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(bytes)).holds<io::envelope::CorruptData>());
    }

    SECTION("an invalid magic")
    {
        io::envelope::Envelope envelope = decode_envelope(io::envelope::serialize<Schema, 3>({}).value());
        envelope.header.magic = 0xF5FBD3EF919428CAULL;
        write_file(path, encode_envelope(envelope));
        CHECK(REQUIRE_ERROR(io::envelope::read_header(path)).holds<io::envelope::CorruptData>());
    }

    SECTION("unknown enumerators")
    {
        io::envelope::Envelope envelope = decode_envelope(io::envelope::serialize<Schema, 3>({}).value());
        SECTION("hash algorithm") { envelope.header.hash_algorithm = static_cast<io::hash::Algorithm>(2); }
        SECTION("compression algorithm") { envelope.header.compression_algorithm = static_cast<io::compression::Algorithm>(3); }
        write_file(path, encode_envelope(envelope));
        CHECK(REQUIRE_ERROR(io::envelope::read_header(path)).holds<Unsupported>());
    }

    SECTION("a missing file") { CHECK(REQUIRE_ERROR(io::envelope::read_header(directory.path() / "missing")).holds<io::FileError>()); }
}

TEST_CASE("Envelope files return and verify their payload hash", "[io][envelope]")
{
    TempDirectory directory("envelope-hash");
    const auto path = directory.path() / "payload.envelope";
    const v3::Payload payload { .id = 5, .label = "identity", .enabled = true, .samples = { 4, 5 } };
    const auto hash = io::envelope::write<Schema>(payload, path, {.mode = io::WriteMode::Overwrite});
    REQUIRE(hash.has_value());
    CHECK(*hash == io::envelope::read_header(path)->hash);
    CHECK(*hash == io::hash::data(encode_value(payload), io::hash::Algorithm::Xxh3_64));
    CHECK(io::envelope::read<Schema>(path, {.expected_hash = *hash}).value() == payload);

    SECTION("a different expected hash")
    {
        auto different = *hash;
        different.front() ^= uint8_t { 1 };
        CHECK(REQUIRE_ERROR(io::envelope::read<Schema>(path, {.expected_hash = different})).holds<io::envelope::CorruptData>());
    }

    SECTION("a hashless payload with an expected hash")
    {
        const auto hashless = io::envelope::write<Schema>(payload, path,
            {.mode = io::WriteMode::Overwrite, .make_dirs = true, .compression_algorithm = io::compression::Algorithm::ZstdDefault, .hash_algorithm = io::hash::Algorithm::None});
        REQUIRE(hashless.has_value());
        CHECK(hashless->empty());
        CHECK(io::envelope::read<Schema>(path).has_value());
        CHECK(REQUIRE_ERROR(io::envelope::read<Schema>(path, {.expected_hash = *hash})).holds<io::envelope::CorruptData>());
    }
}
