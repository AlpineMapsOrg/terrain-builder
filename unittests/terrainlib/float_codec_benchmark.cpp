/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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

#include "../temporary_directory.h"
#include "raster_store/io/TileCodec.h"

#include <cstdlib>
#include <cstring>

#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <fmt/format.h>

// Explicit opt-in: AMOFT_BENCHMARK_TILE=/path/to/dsm.amort
// unittests_terrainlib '[amoft-benchmark]' --benchmark-samples 5
TEST_CASE("AMORT versus AMOFT on identical Float32 RF data", "[.][amoft-benchmark]")
{
    namespace codecs = raster_store::io::tile_codec;
    using io::envelope::CompressionAlgorithm;
    const char* input = std::getenv("AMOFT_BENCHMARK_TILE");
    REQUIRE(input != nullptr);
    const auto source = io::envelope::read<codecs::TileSchema>(input);
    REQUIRE(source);
    REQUIRE(source->width > 0);
    REQUIRE(source->width == source->height);
    REQUIRE(source->data.size() == std::uint64_t(source->width) * source->height * sizeof(float));
    REQUIRE(source->source_attribution.size() == std::uint64_t(source->width) * source->height * sizeof(std::uint16_t));
    const auto predicted = GENERATE(false, true);
    const auto compression = GENERATE(CompressionAlgorithm::ZstdDefaultCompression, CompressionAlgorithm::ZstdBestCompression);
    const auto label
        = fmt::format("{} / {}", predicted ? "AMOFT" : "AMORT", compression == CompressionAlgorithm::ZstdDefaultCompression ? "zstd" : "zstd-best");
    CAPTURE(input, label);

    // Match the codec's copies, prediction, serialization, hashing, compression,
    // allocation, and raster reconstruction, excluding filesystem operations.
    const auto encode = [&] {
        auto payload = *source;
        if (predicted) {
            raster_store::io::float_predictor::encode(payload.data, payload.width);
            return Error::throwing_unwrap(io::envelope::serialize<codecs::FloatTileSchema>(payload, compression));
        }
        return Error::throwing_unwrap(io::envelope::serialize<codecs::TileSchema>(payload, compression));
    };
    const auto encoded = encode();
    const auto decode = [&] {
        auto payload = Error::throwing_unwrap(
            predicted ? io::envelope::deserialize<codecs::FloatTileSchema>(encoded) : io::envelope::deserialize<codecs::TileSchema>(encoded));
        raster_store::Tile<float> tile(payload.width);
        if (predicted) {
            raster_store::io::float_predictor::decode(payload.data, payload.width);
        }
        std::memcpy(tile.data.bytes().data(), payload.data.data(), payload.data.size());
        std::memcpy(tile.attribution.bytes().data(), payload.source_attribution.data(), payload.source_attribution.size());
        return tile;
    };
    const auto decoded = decode();
    REQUIRE(std::ranges::equal(decoded.data.bytes(), source->data));
    REQUIRE(std::ranges::equal(decoded.attribution.bytes(), source->source_attribution));

    // Check that this in-memory benchmark produces the actual codec file bytes.
    test::TemporaryDirectory directory;
    const auto codec = codecs::from_name<float>(predicted ? "amoft" : "amort", glm::uvec2(source->width), compression);
    REQUIRE(codec);
    const auto path = directory.path() / "tile";
    const auto hash = (*codec)->write(path, decoded);
    REQUIRE(hash);
    REQUIRE(io::byte::read((*codec)->paths(path).front()).value() == encoded);
    const auto roundtrip = (*codec)->read(path, *hash);
    REQUIRE(roundtrip);
    REQUIRE(std::ranges::equal(roundtrip->data.bytes(), source->data));
    REQUIRE(std::ranges::equal(roundtrip->attribution.bytes(), source->source_attribution));
    fmt::print("\n{}: {}x{}, {} bytes, bit-exact data and attribution\n", label, source->width, source->height, encoded.size());
    BENCHMARK(label + " encode") { return encode(); };
    BENCHMARK(label + " decode") { return decode(); };
}
