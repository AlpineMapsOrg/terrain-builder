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

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <random>
#include <ranges>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_precision.hpp>

#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "common_error_codes.h"
#include "io/FileError.h"
#include "io/WriteMode.h"
#include "io/bytes.h"
#include "io/compression.h"
#include "io/envelope.h"
#include "io/image.h"
#include "io/serde.h"
#include "raster_store/Tile.h"
#include "raster_store/io/TileCodec.h"
#include "raster_store_test_utils.h"
#include "store/Codec.h"
#include "store/Hash.h"

namespace {

namespace tile_codec = raster_store::io::tile_codec;
using raster_store::io::TileCodec;
using raster_store_test::check_bytes;

// What the tests of each image codec need to know about it.
struct Amoit {
    static constexpr tile_codec::DataEncoding encoding = tile_codec::DataEncoding::JpegXl;
    using Schema = tile_codec::ImageTileSchema;
    static constexpr io::image::Format format = io::image::Format::JpegXl;
    static constexpr std::string_view extension = ".amoit";
    static constexpr int max_error = 4;
};

struct Amojt {
    static constexpr tile_codec::DataEncoding encoding = tile_codec::DataEncoding::Jpeg;
    using Schema = tile_codec::JpegTileSchema;
    static constexpr io::image::Format format = io::image::Format::Jpeg;
    static constexpr std::string_view extension = ".amojt";
    static constexpr int max_error = 6;
};

} // namespace

TEMPLATE_TEST_CASE("AMORT preserves native scalar and packed vector bytes", "[raster_store][io][TileCodec]",
    float, double, int16_t, uint32_t, glm::vec3, glm::u8vec3)
{
    const TempDirectory directory("amort-bytes");
    raster_store::Tile<TestType> tile(3);
    CHECK(std::ranges::all_of(tile.attribution.buffer(), [](const uint16_t value) { return value == 0; }));
    uint8_t value = 0;
    for (std::byte& byte : tile.data.bytes()) {
        byte = std::byte { value };
        value = static_cast<uint8_t>(value + 37);
    }
    tile.attribution.pixel({ 1, 1 }) = 65534;
    const std::filesystem::path path = directory.path() / "tile";
    const TileCodec<TestType> writer(3);
    REQUIRE_VALUE(writer.write(path, tile));
    const raster_store::Tile<TestType> read = REQUIRE_VALUE(writer.read(path));
    check_bytes(read.data, tile.data);
    check_bytes(read.attribution, tile.attribution);

    const TileCodec<TestType> uncompressed(3, io::compression::Algorithm::None);
    const std::filesystem::path uncompressed_path = directory.path() / "uncompressed";
    REQUIRE_VALUE(uncompressed.write(uncompressed_path, tile));
    check_bytes(REQUIRE_VALUE(writer.read(uncompressed_path)).data, tile.data);
}

TEST_CASE("AMORT never replaces an existing tile file", "[raster_store][io][TileCodec]")
{
    const TempDirectory directory("amort-exclusive");
    const std::filesystem::path path = directory.path() / "tile";
    const TileCodec<float> amort(3);
    raster_store::Tile<float> tile(3);
    tile.data.pixel({ 0, 0 }) = 1.0f;
    REQUIRE_VALUE(amort.write(path, tile));
    const std::filesystem::path file = amort.paths(path).front();
    const io::bytes::vector original = REQUIRE_VALUE(io::bytes::read(file));

    tile.data.pixel({ 0, 0 }) = 2.0f;
    const auto error = REQUIRE_ERROR(amort.write(path, tile));
    CHECK(error.holds_and<io::FileError>([](const io::FileError& e) { return e.code() == std::errc::file_exists; }));
    CHECK(REQUIRE_VALUE(io::bytes::read(file)) == original);
}

TEST_CASE("AMORT preserves NaN bits and data beneath NoData pixels", "[raster_store][io][TileCodec]")
{
    const TempDirectory directory("amort-nan");
    raster_store::Tile<float> tile(3);
    tile.data.pixel({ 0, 0 }) = std::bit_cast<float>(uint32_t { 0x7fc01234 });
    tile.attribution.pixel({ 0, 0 }) = 1;
    tile.data.pixel({ 1, 0 }) = 42.0f;
    tile.data.pixel({ 2, 0 }) = std::bit_cast<float>(uint32_t { 0x80000000 });
    const TileCodec<float> amort(3);
    REQUIRE_VALUE(amort.write(directory.path() / "tile", tile));
    const raster_store::Tile<float> read = REQUIRE_VALUE(amort.read(directory.path() / "tile"));
    check_bytes(read.data, tile.data);
    check_bytes(read.attribution, tile.attribution);
}

TEST_CASE("AMORT validates dimensions and byte counts before allocating typed rasters", "[raster_store][io][TileCodec]")
{
    const TempDirectory directory("amort-validation");
    const std::filesystem::path path = directory.path() / "tile";
    const TileCodec<float> amort(3);

    tile_codec::detail::v1::RasterTile encoded { 3, 3, io::bytes::vector(36), io::bytes::vector(18) };
    SECTION("wrong dimensions") { encoded.width = 4; }
    SECTION("short data") { encoded.data.pop_back(); }
    SECTION("long attribution") { encoded.source_attribution.push_back(0); }
    REQUIRE_VALUE(io::envelope::write<tile_codec::TileSchema>(encoded, amort.paths(path).front(), { .mode = io::WriteMode::Overwrite }));
    CHECK(REQUIRE_ERROR(amort.read(path)).holds<store::CorruptData>());
}

TEMPLATE_TEST_CASE("image tile codecs store RGB8 data lossily and attribution exactly", "[raster_store][io][TileCodec]", Amoit, Amojt)
{
    const TempDirectory directory("image-lossy");
    raster_store::Tile<glm::u8vec3> tile(32);
    for (uint32_t y = 0; y < 32; y++) {
        for (uint32_t x = 0; x < 32; x++) {
            tile.data.pixel({ x, y }) = glm::u8vec3(100 + 2 * x, 80 + 2 * y, 120);
            tile.attribution.pixel({ x, y }) = static_cast<uint16_t>(x + 32 * y);
        }
    }
    tile.attribution.pixel({ 31, 31 }) = 65534;
    const std::filesystem::path path = directory.path() / "tile";
    const TileCodec<glm::u8vec3, TestType::encoding> codec(32);
    CHECK(codec.paths(path) == std::vector { std::filesystem::path(path.string() + std::string(TestType::extension)) });
    const store::PayloadHash hash = REQUIRE_VALUE(codec.write(path, tile));
    CHECK(hash.size() >= store::min_hash_size);
    const raster_store::Tile<glm::u8vec3> read = REQUIRE_VALUE(codec.read(path, hash));
    check_bytes(read.attribution, tile.attribution);
    REQUIRE(read.data.size() == tile.data.size());
    int max_error = 0;
    for (const auto [read_pixel, pixel] : std::views::zip(read.data.buffer(), tile.data.buffer())) {
        const glm::ivec3 difference = glm::abs(glm::ivec3(read_pixel) - glm::ivec3(pixel));
        max_error = std::max({ max_error, difference.x, difference.y, difference.z });
    }
    CHECK(max_error > 0);
    CHECK(max_error <= TestType::max_error);
    const auto error = REQUIRE_ERROR(codec.write(path, tile));
    CHECK(error.template holds_and<io::FileError>([](const io::FileError& e) { return e.code() == std::errc::file_exists; }));
}

TEST_CASE("AMOJT stores the JPEG that io::image encodes for the tile", "[raster_store][io][TileCodec]")
{
    const TempDirectory directory("amojt-payload");
    raster_store::Tile<glm::u8vec3> tile(8);
    for (uint32_t y = 0; y < 8; y++) {
        for (uint32_t x = 0; x < 8; x++) {
            tile.data.pixel({ x, y }) = glm::u8vec3(30 * x, 30 * y, 90);
        }
    }
    const std::filesystem::path path = directory.path() / "tile";
    const TileCodec<glm::u8vec3, tile_codec::DataEncoding::Jpeg> amojt(8);
    REQUIRE_VALUE(amojt.write(path, tile));
    const tile_codec::detail::v1::RasterTile encoded = REQUIRE_VALUE(io::envelope::read<tile_codec::JpegTileSchema>(amojt.paths(path).front()));
    CHECK(encoded.data == REQUIRE_VALUE(io::image::encode(tile.data, io::image::Format::Jpeg)));
}

TEMPLATE_TEST_CASE("image tile codecs validate images and attribution against the tile dimensions", "[raster_store][io][TileCodec]", Amoit, Amojt)
{
    const TempDirectory directory("image-validation");
    const std::filesystem::path path = directory.path() / "tile";
    const TileCodec<glm::u8vec3, TestType::encoding> codec(3);
    const std::filesystem::path file = codec.paths(path).front();
    tile_codec::detail::v1::RasterTile encoded { 3, 3, REQUIRE_VALUE(io::image::encode(io::image::RGB8(3), TestType::format)), io::bytes::vector(18) };
    bool valid = false;
    SECTION("valid") { valid = true; }
    SECTION("wrong dimensions") { encoded.width = 4; }
    SECTION("image of other dimensions") { encoded.data = REQUIRE_VALUE(io::image::encode(io::image::RGB8(4), TestType::format)); }
    SECTION("truncated image") { encoded.data.resize(encoded.data.size() / 2); }
    SECTION("long attribution") { encoded.source_attribution.push_back(0); }
    REQUIRE_VALUE(io::envelope::write<typename TestType::Schema>(encoded, file, { .mode = io::WriteMode::Overwrite }));
    if (valid) {
        REQUIRE_VALUE(codec.read(path));
    } else {
        CHECK(REQUIRE_ERROR(codec.read(path)).template holds<store::CorruptData>());
    }

    // AMORT payloads use another envelope class.
    REQUIRE_VALUE(io::envelope::write<tile_codec::TileSchema>(encoded, file, { .mode = io::WriteMode::Overwrite }));
    CHECK(REQUIRE_ERROR(codec.read(path)).template holds<store::CorruptData>());
}

TEST_CASE("AMOFT preserves all Float32 bits and attribution with both Zstd settings", "[raster_store][io][TileCodec]")
{
    const uint32_t side = GENERATE(1u, 3u, 17u, 64u);
    const io::compression::Algorithm compression = GENERATE(io::compression::Algorithm::ZstdDefault, io::compression::Algorithm::ZstdBest);
    const TempDirectory directory("amoft-bits");
    raster_store::Tile<float> tile(side);
    std::mt19937 random(42);
    std::ranges::generate(tile.data.buffer(), [&random]() { return std::bit_cast<float>(static_cast<uint32_t>(random())); });
    // Zeros, infinities, NaNs with payloads, denormals and the extremes.
    constexpr std::array<uint32_t, 12> special_bits = {
        0x00000000, 0x80000000, 0x7f800000, 0xff800000, 0x7fc01234, 0xffc05678, 0x7f801234, 0x00000001, 0x80000001, 0x007fffff, 0x00800000, 0x7f7fffff
    };
    for (auto [value, bits] : std::views::zip(tile.data.buffer(), special_bits)) {
        value = std::bit_cast<float>(bits);
    }
    // Values under NoData must also keep every bit.
    tile.attribution.fill(0);
    tile.attribution.pixel({ side - 1, 0 }) = 65534;
    const radix::Raster<float> original = tile.data;

    const std::filesystem::path path = directory.path() / "tile";
    const TileCodec<float, tile_codec::DataEncoding::FloatPredictor> writer(side, compression);
    const store::PayloadHash hash = REQUIRE_VALUE(writer.write(path, tile));
    check_bytes(tile.data, original);
    CHECK(writer.paths(path).front().extension() == ".amoft");
    const io::envelope::Header header = REQUIRE_VALUE(io::envelope::read_header(writer.paths(path).front()));
    CHECK(header.class_name == "raster_store.FloatTile");
    CHECK(header.compression_algorithm == compression);

    // The reader takes the compression from the envelope.
    const TileCodec<float, tile_codec::DataEncoding::FloatPredictor> reader(side);
    const raster_store::Tile<float> read = REQUIRE_VALUE(reader.read(path, hash));
    check_bytes(read.data, original);
    check_bytes(read.attribution, tile.attribution);
    const auto error = REQUIRE_ERROR(writer.write(path, tile));
    CHECK(error.holds_and<io::FileError>([](const io::FileError& e) { return e.code() == std::errc::file_exists; }));
}

TEST_CASE("AMOFT rejects wrong dimensions, byte counts and envelope classes", "[raster_store][io][TileCodec]")
{
    const TempDirectory directory("amoft-validation");
    const std::filesystem::path path = directory.path() / "tile";
    const TileCodec<float, tile_codec::DataEncoding::FloatPredictor> amoft(3);
    const std::filesystem::path file = amoft.paths(path).front();
    tile_codec::detail::v1::RasterTile encoded { 3, 3, io::bytes::vector(36), io::bytes::vector(18) };
    SECTION("wrong width") { encoded.width = 4; }
    SECTION("no dimensions")
    {
        encoded.width = 0;
        encoded.height = 0;
    }
    SECTION("empty data") { encoded.data.clear(); }
    SECTION("short data") { encoded.data.pop_back(); }
    SECTION("long data") { encoded.data.push_back(0); }
    SECTION("short attribution") { encoded.source_attribution.pop_back(); }
    REQUIRE_VALUE(io::envelope::write<tile_codec::FloatTileSchema>(encoded, file, { .mode = io::WriteMode::Overwrite }));
    CHECK(REQUIRE_ERROR(amoft.read(path)).holds<store::CorruptData>());

    // AMORT payloads use another envelope class.
    REQUIRE_VALUE(io::envelope::write<tile_codec::TileSchema>(encoded, file, { .mode = io::WriteMode::Overwrite }));
    CHECK(REQUIRE_ERROR(amoft.read(path)).holds<store::CorruptData>());

    CHECK_THROWS(amoft.write(directory.path() / "other", raster_store::Tile<float>(4)));
}

TEST_CASE("AMORT propagates envelope checksum and version errors", "[raster_store][io][TileCodec]")
{
    const TempDirectory directory("amort-envelope");
    const std::filesystem::path path = directory.path() / "tile";
    const TileCodec<float> amort(3);
    REQUIRE_VALUE(amort.write(path, raster_store::Tile<float>(3)));
    const std::filesystem::path file = amort.paths(path).front();
    io::envelope::Envelope envelope = REQUIRE_VALUE(io::serde::deserialize<io::envelope::Envelope>(REQUIRE_VALUE(io::bytes::read(file))));

    SECTION("hash")
    {
        envelope.compressed_data.back() ^= 1;
        REQUIRE_VALUE(io::bytes::write(REQUIRE_VALUE(io::serde::serialize(envelope)), file, { .mode = io::WriteMode::Overwrite }));
        CHECK(REQUIRE_ERROR(amort.read(path)).holds<store::CorruptData>());
    }

    SECTION("version")
    {
        envelope.header.class_version = 999;
        REQUIRE_VALUE(io::bytes::write(REQUIRE_VALUE(io::serde::serialize(envelope)), file, { .mode = io::WriteMode::Overwrite }));
        CHECK(REQUIRE_ERROR(amort.read(path)).holds<Unsupported>());
    }
}

TEST_CASE("AMORT tiles return and verify their payload hash", "[raster_store][io][TileCodec]")
{
    const TempDirectory directory("amort-hash");
    const std::filesystem::path path = directory.path() / "tile";
    raster_store::Tile<float> tile(3);
    tile.data.fill(2.5f);
    const TileCodec<float> amort(3);
    const store::PayloadHash hash = REQUIRE_VALUE(amort.write(path, tile));
    CHECK(hash.size() == 8);
    CHECK(hash == REQUIRE_VALUE(io::envelope::read_header(amort.paths(path).front())).hash);
    REQUIRE_VALUE(amort.read(path, hash));

    store::PayloadHash different = hash;
    different.front() ^= 1;
    CHECK(REQUIRE_ERROR(amort.read(path, different)).holds<store::CorruptData>());
}
