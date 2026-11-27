/*****************************************************************************
 * AlpineMaps.org
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

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <cstdint>
#include <memory>
#include <numeric>

#include <zstd.h>

#include "../result_helpers.h"
#include "io/bytes.h"
#include "io/compression.h"

namespace {

using io::compression::Algorithm;

// Data of `size` bytes.
io::bytes::vector make_data_bytes(const size_t size) {
    io::bytes::vector data(size);
    std::ranges::iota(data, uint8_t{0});
    return data;
}

// Compresses `data` into a zstd frame that does not record its content size.
io::bytes::vector compress_without_content_size(const io::bytes::span data) {
    const std::unique_ptr<ZSTD_CCtx, decltype(&ZSTD_freeCCtx)> context{ZSTD_createCCtx(), &ZSTD_freeCCtx};
    REQUIRE(context);
    REQUIRE_FALSE(ZSTD_isError(ZSTD_CCtx_setParameter(context.get(), ZSTD_c_contentSizeFlag, 0)));

    io::bytes::vector compressed_data(ZSTD_compressBound(data.size()));
    const size_t compressed_size = ZSTD_compress2(context.get(), compressed_data.data(), compressed_data.size(), data.data(), data.size());
    REQUIRE_FALSE(ZSTD_isError(compressed_size));
    compressed_data.resize(compressed_size);
    return compressed_data;
}

} // namespace

TEST_CASE("compress and decompress round-trip data", "[io][compression]") {
    const Algorithm algorithm = GENERATE(Algorithm::None, Algorithm::ZstdBest, Algorithm::ZstdDefault);

    SECTION("4096 bytes") {
        const io::bytes::vector data = make_data_bytes(4096);
        const io::bytes::vector compressed = REQUIRE_VALUE(io::compression::compress(data, algorithm));

        CHECK(REQUIRE_VALUE(io::compression::decompress(compressed, algorithm)) == data);
    }

    SECTION("empty data") {
        const io::bytes::vector compressed = REQUIRE_VALUE(io::compression::compress({}, algorithm));

        CHECK(REQUIRE_VALUE(io::compression::decompress(compressed, algorithm)) == io::bytes::vector{});
    }
}

TEST_CASE("decompress limits the output size", "[io][compression]") {
    const io::bytes::vector data = make_data_bytes(4096);

    SECTION("with the content size") {
        const Algorithm algorithm = GENERATE(Algorithm::None, Algorithm::ZstdBest, Algorithm::ZstdDefault);
        const io::bytes::vector compressed = REQUIRE_VALUE(io::compression::compress(data, algorithm));

        CHECK(REQUIRE_VALUE(io::compression::decompress(compressed, algorithm, 4096)) == data);
        CHECK(REQUIRE_ERROR(io::compression::decompress(compressed, algorithm, 4095)).holds<io::BufferLimitExceeded>());
    }

    SECTION("without the content size") {
        const Algorithm algorithm = GENERATE(Algorithm::ZstdBest, Algorithm::ZstdDefault);
        const io::bytes::vector compressed = compress_without_content_size(data);

        CHECK(REQUIRE_VALUE(io::compression::decompress(compressed, algorithm, 4096)) == data);
        CHECK(REQUIRE_ERROR(io::compression::decompress(compressed, algorithm, 4095)).holds<io::BufferLimitExceeded>());
    }
}

TEST_CASE("decompress rejects corrupt data", "[io][compression]") {
    const Algorithm algorithm = GENERATE(Algorithm::ZstdBest, Algorithm::ZstdDefault);

    SECTION("not a zstd frame") {
        const io::bytes::vector data = {1, 2, 3, 4};

        CHECK(REQUIRE_ERROR(io::compression::decompress(data, algorithm)).holds<io::compression::CorruptData>());
    }

    SECTION("truncated frame") {
        io::bytes::vector compressed = REQUIRE_VALUE(io::compression::compress(make_data_bytes(4096), algorithm));
        compressed.pop_back();

        CHECK(REQUIRE_ERROR(io::compression::decompress(compressed, algorithm)).holds<io::compression::CorruptData>());
    }
}
