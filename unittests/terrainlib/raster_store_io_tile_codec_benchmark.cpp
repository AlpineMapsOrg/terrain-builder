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

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <span>
#include <string>

#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <fmt/format.h>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "error/unwrap.h"
#include "io/bytes.h"
#include "io/compression.h"
#include "io/envelope.h"
#include "raster_store/Tile.h"
#include "raster_store/io/TileCodec.h"
#include "raster_store/io/float_predictor.h"
#include "store/Hash.h"

namespace {

namespace tile_codec = raster_store::io::tile_codec;
using Compression = io::compression::Algorithm;

} // namespace

// Opt in with AMOFT_BENCHMARK_TILE=/path/to/dsm.amort, then run: unittests_terrainlib "AMORT versus AMOFT on identical Float32 tiles" --benchmark-samples 5
TEST_CASE("AMORT versus AMOFT on identical Float32 tiles", "[raster_store][io][TileCodec][!benchmark][.]")
{
    const char *input = std::getenv("AMOFT_BENCHMARK_TILE");
    REQUIRE(input != nullptr);
    const tile_codec::detail::v1::RasterTile source = REQUIRE_VALUE(io::envelope::read<tile_codec::TileSchema>(input));
    REQUIRE(source.width > 0);
    REQUIRE(source.width == source.height);
    REQUIRE(source.data.size() == uint64_t{source.width} * source.height * sizeof(float));
    REQUIRE(source.source_attribution.size() == uint64_t{source.width} * source.height * sizeof(uint16_t));
    const bool predicted = GENERATE(false, true);
    const Compression compression = GENERATE(Compression::ZstdDefault, Compression::ZstdBest);
    const std::string label = fmt::format("{} / {}", predicted ? "AMOFT" : "AMORT", compression == Compression::ZstdDefault ? "zstd" : "zstd-best");
    CAPTURE(input, label);

    // Matches the codec's copies, prediction, serialization, hashing, compression, allocation and raster reconstruction, without the filesystem.
    const auto encode = [&]() {
        tile_codec::detail::v1::RasterTile payload = source;
        if (predicted) {
            raster_store::io::float_predictor::encode(std::as_writable_bytes(std::span(payload.data)), payload.width);
            return alp::throwing_unwrap(io::envelope::serialize<tile_codec::FloatTileSchema>(payload, { .compression_algorithm = compression }));
        } else {
            return alp::throwing_unwrap(io::envelope::serialize<tile_codec::TileSchema>(payload, { .compression_algorithm = compression }));
        }
    };
    const io::bytes::vector encoded = encode();
    const auto decode = [&]() {
        tile_codec::detail::v1::RasterTile payload = predicted ? alp::throwing_unwrap(io::envelope::deserialize<tile_codec::FloatTileSchema>(encoded))
                                                               : alp::throwing_unwrap(io::envelope::deserialize<tile_codec::TileSchema>(encoded));
        raster_store::Tile<float> tile(payload.width);
        if (predicted) {
            raster_store::io::float_predictor::decode(std::as_writable_bytes(std::span(payload.data)), payload.width);
        }
        std::memcpy(tile.data.bytes().data(), payload.data.data(), payload.data.size());
        std::memcpy(tile.attribution.bytes().data(), payload.source_attribution.data(), payload.source_attribution.size());
        return tile;
    };
    const raster_store::Tile<float> decoded = decode();
    REQUIRE(std::ranges::equal(std::as_bytes(decoded.data.bytes()), std::as_bytes(std::span(source.data))));
    REQUIRE(std::ranges::equal(std::as_bytes(decoded.attribution.bytes()), std::as_bytes(std::span(source.source_attribution))));

    // The in-memory encoding is what the codec writes to its file.
    const TempDirectory directory("amoft-benchmark");
    const auto codec = REQUIRE_VALUE(tile_codec::resolve<float>(predicted ? tile_codec::amoft_id : tile_codec::amort_id, source.width, compression));
    const std::filesystem::path path = directory.path() / "tile";
    const store::PayloadHash hash = REQUIRE_VALUE(codec->write(path, decoded));
    REQUIRE(REQUIRE_VALUE(io::bytes::read(codec->paths(path).front())) == encoded);
    const raster_store::Tile<float> round_trip = REQUIRE_VALUE(codec->read(path, hash));
    REQUIRE(std::ranges::equal(std::as_bytes(round_trip.data.bytes()), std::as_bytes(std::span(source.data))));
    REQUIRE(std::ranges::equal(std::as_bytes(round_trip.attribution.bytes()), std::as_bytes(std::span(source.source_attribution))));
    fmt::print("\n{}: {}x{}, {} bytes, bit-exact data and attribution\n", label, source.width, source.height, encoded.size());

    BENCHMARK(label + " encode") { return encode(); };
    BENCHMARK(label + " decode") { return decode(); };
}
