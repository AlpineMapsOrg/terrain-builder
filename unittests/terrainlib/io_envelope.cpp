#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <system_error>
#include <vector>

#include <zpp_bits.h>

#include "../result_helpers.h"
#include "io/SerdeError.h"
#include "io/envelope.h"

namespace {

namespace v1 {

struct Payload {
    std::uint32_t id;
    std::string name;

    bool operator==(const Payload &) const = default;
};

} // namespace v1

namespace v2 {

struct Payload {
    std::uint64_t id;
    std::string name;
    bool enabled;

    static Payload from_previous(v1::Payload previous) {
        return {
            .id = previous.id,
            .name = std::move(previous.name),
            .enabled = true,
        };
    }

    bool operator==(const Payload &) const = default;
};

} // namespace v2

namespace v3 {

struct Payload {
    std::uint64_t id;
    std::string label;
    bool enabled;
    std::vector<std::int32_t> samples;

    static Payload from_previous(v2::Payload previous) {
        return {
            .id = previous.id,
            .label = std::move(previous.name),
            .enabled = previous.enabled,
            .samples = {},
        };
    }

    bool operator==(const Payload &) const = default;
};

} // namespace v3

using Schema = io::envelope::PayloadSchema<
    "test.Payload",
    io::envelope::Version<1, v1::Payload>,
    io::envelope::Version<2, v2::Payload>,
    io::envelope::Version<3, v3::Payload>>;

static_assert(Schema::class_name == "test.Payload");
static_assert(Schema::latest_version == 3);
static_assert(std::same_as<Schema::Latest, v3::Payload>);
static_assert(std::same_as<Schema::Payload<1>, v1::Payload>);

using io::envelope::Bytes;
using io::envelope::ChecksumAlgorithm;
using io::envelope::CompressionAlgorithm;
using io::envelope::CompressionError;
using io::envelope::EnvelopeError;

template <typename Value>
Bytes encode_value(const Value &value) {
    Bytes bytes;
    zpp::bits::out output(bytes);
    output(value).or_throw();
    return bytes;
}

io::envelope::Envelope decode_envelope(const Bytes &bytes) {
    io::envelope::Envelope envelope{};
    zpp::bits::in input(bytes);
    input(envelope).or_throw();
    return envelope;
}

} // namespace

TEST_CASE("serialize writes the envelope metadata", "[io][envelope]") {
    const v3::Payload payload{.id = 42, .label = "latest", .enabled = false, .samples = {1, 2, 3}};

    const io::envelope::Envelope envelope = decode_envelope(REQUIRE_VALUE(io::envelope::serialize<Schema, 3>(payload)));

    CHECK(envelope.magic == io::envelope::magic);
    CHECK(envelope.class_name == "test.Payload");
    CHECK(envelope.class_version == 3);
    CHECK(envelope.checksum_algorithm == ChecksumAlgorithm::HandledByCompressionLib);
    CHECK(envelope.checksum == "");
    CHECK(envelope.compression_algorithm == CompressionAlgorithm::ZstdBestCompressionWithChecksum);
    CHECK(envelope.uncompressed_size == encode_value(payload).size());
}

TEST_CASE("deserialize restores payloads written with each algorithm pair", "[io][envelope]") {
    const auto [compression_algorithm, checksum_algorithm] = GENERATE(table<CompressionAlgorithm, ChecksumAlgorithm>({
        {CompressionAlgorithm::None, ChecksumAlgorithm::None},
        {CompressionAlgorithm::None, ChecksumAlgorithm::Crc32c},
        {CompressionAlgorithm::ZstdBestCompressionWithChecksum, ChecksumAlgorithm::HandledByCompressionLib},
        {CompressionAlgorithm::ZstdBestCompressionWithChecksum, ChecksumAlgorithm::Crc32c},
    }));
    const v3::Payload payload{.id = 42, .label = "latest", .enabled = false, .samples = {1, 2, 3}};

    const Bytes bytes = REQUIRE_VALUE(io::envelope::serialize<Schema, 3>(payload, compression_algorithm, checksum_algorithm));
    const io::envelope::Envelope envelope = decode_envelope(bytes);

    CHECK(envelope.compression_algorithm == compression_algorithm);
    CHECK(envelope.checksum_algorithm == checksum_algorithm);
    CHECK(REQUIRE_VALUE(io::envelope::deserialize<Schema>(bytes)) == payload);
}

TEST_CASE("deserialize upgrades older payload versions to the latest", "[io][envelope]") {
    SECTION("version 1") {
        const v1::Payload payload{.id = 7, .name = "version one"};
        const Bytes bytes = REQUIRE_VALUE(io::envelope::serialize<Schema, 1>(payload));

        CHECK(REQUIRE_VALUE(io::envelope::deserialize<Schema>(bytes)) == v3::Payload{.id = 7, .label = "version one", .enabled = true, .samples = {}});
    }

    SECTION("version 2") {
        const v2::Payload payload{.id = 9, .name = "version two", .enabled = false};
        const Bytes bytes = REQUIRE_VALUE(io::envelope::serialize<Schema, 2>(payload));

        CHECK(REQUIRE_VALUE(io::envelope::deserialize<Schema>(bytes)) == v3::Payload{.id = 9, .label = "version two", .enabled = false, .samples = {}});
    }
}

TEST_CASE("deserialize rejects incompatible metadata", "[io][envelope]") {
    const v3::Payload payload{.id = 42, .label = "latest", .enabled = false, .samples = {1, 2, 3}};
    io::envelope::Envelope envelope = decode_envelope(REQUIRE_VALUE(io::envelope::serialize<Schema, 3>(payload)));

    SECTION("magic") {
        envelope.magic++;

        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_value(envelope))).holds(EnvelopeError::InvalidMagic));
    }

    SECTION("class name") {
        envelope.class_name = "other.Payload";

        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_value(envelope))).holds(EnvelopeError::WrongClassName));
    }

    SECTION("class version") {
        envelope.class_version = 99;

        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_value(envelope))).holds(EnvelopeError::UnsupportedClassVersion));
    }
}

TEST_CASE("deserialize passes the envelope's algorithms and checksum to decompression", "[io][envelope]") {
    const v3::Payload payload{.id = 42, .label = "latest", .enabled = false, .samples = {1, 2, 3}};

    SECTION("checksum algorithm") {
        io::envelope::Envelope envelope = decode_envelope(REQUIRE_VALUE(io::envelope::serialize<Schema, 3>(payload)));
        envelope.checksum_algorithm = static_cast<ChecksumAlgorithm>(99);

        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_value(envelope))).holds(CompressionError::UnsupportedChecksumAlgorithm));
    }

    SECTION("compression algorithm") {
        io::envelope::Envelope envelope = decode_envelope(REQUIRE_VALUE(io::envelope::serialize<Schema, 3>(payload)));
        envelope.compression_algorithm = static_cast<CompressionAlgorithm>(99);

        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_value(envelope))).holds(CompressionError::UnsupportedCompressionAlgorithm));
    }

    SECTION("checksum") {
        io::envelope::Envelope envelope = decode_envelope(REQUIRE_VALUE(io::envelope::serialize<Schema, 3>(payload)));
        envelope.checksum = "e3069283";

        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_value(envelope))).holds(CompressionError::InvalidAlgorithmCombination));
    }

    SECTION("data") {
        io::envelope::Envelope envelope = decode_envelope(REQUIRE_VALUE(io::envelope::serialize<Schema, 3>(payload, CompressionAlgorithm::None, ChecksumAlgorithm::Crc32c)));
        envelope.compressed_data.front() ^= std::byte{0x01};

        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_value(envelope))).holds(CompressionError::ChecksumMismatch));
    }
}

TEST_CASE("deserialize rejects a declared size that does not match the payload", "[io][envelope]") {
    const auto [compression_algorithm, checksum_algorithm] = GENERATE(table<CompressionAlgorithm, ChecksumAlgorithm>({
        {CompressionAlgorithm::None, ChecksumAlgorithm::None},
        {CompressionAlgorithm::ZstdBestCompressionWithChecksum, ChecksumAlgorithm::HandledByCompressionLib},
    }));
    const v3::Payload payload{.id = 42, .label = "latest", .enabled = false, .samples = {1, 2, 3}};
    io::envelope::Envelope envelope = decode_envelope(REQUIRE_VALUE(io::envelope::serialize<Schema, 3>(payload, compression_algorithm, checksum_algorithm)));

    SECTION("smaller") {
        envelope.uncompressed_size--;

        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_value(envelope))).holds(EnvelopeError::UncompressedSizeMismatch));
    }

    SECTION("larger") {
        envelope.uncompressed_size++;

        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_value(envelope))).holds(EnvelopeError::UncompressedSizeMismatch));
    }

    SECTION("zero") {
        envelope.uncompressed_size = 0;

        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_value(envelope))).holds(EnvelopeError::UncompressedSizeMismatch));
    }
}

TEST_CASE("deserialize enforces its size limits", "[io][envelope]") {
    const v3::Payload payload{.id = 42, .label = "latest", .enabled = false, .samples = {1, 2, 3}};
    const size_t payload_size = encode_value(payload).size();
    const Bytes bytes = REQUIRE_VALUE(io::envelope::serialize<Schema, 3>(payload));

    SECTION("hard limit") {
        io::envelope::Envelope envelope = decode_envelope(bytes);
        envelope.uncompressed_size = io::envelope::default_max_decompressed_size + 1;

        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_value(envelope))).holds(EnvelopeError::SizeLimitExceeded));
    }

    SECTION("caller limit") {
        CHECK(REQUIRE_VALUE(io::envelope::deserialize<Schema>(bytes, payload_size)) == payload);
        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(bytes, payload_size - 1)).holds(EnvelopeError::SizeLimitExceeded));
    }
}

TEST_CASE("deserialize rejects malformed data", "[io][envelope]") {
    SECTION("envelope ends early") {
        const Bytes bytes{std::byte{0x01}, std::byte{0x02}};

        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(bytes)).holds(io::SerdeError(std::errc::result_out_of_range)));
    }

    SECTION("payload ends early") {
        const io::envelope::Envelope envelope{
            .magic = io::envelope::magic,
            .class_name = "test.Payload",
            .class_version = 3,
            .checksum_algorithm = ChecksumAlgorithm::None,
            .checksum = "",
            .compression_algorithm = CompressionAlgorithm::None,
            .uncompressed_size = 1,
            .compressed_data = {std::byte{0x01}},
        };

        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(encode_value(envelope))).holds(io::SerdeError(std::errc::result_out_of_range)));
    }

    SECTION("trailing data") {
        const v3::Payload payload{.id = 42, .label = "latest", .enabled = false, .samples = {1, 2, 3}};
        Bytes bytes = REQUIRE_VALUE(io::envelope::serialize<Schema, 3>(payload));
        bytes.push_back(std::byte{0x00});

        CHECK(REQUIRE_ERROR(io::envelope::deserialize<Schema>(bytes)).holds(EnvelopeError::TrailingData));
    }
}
