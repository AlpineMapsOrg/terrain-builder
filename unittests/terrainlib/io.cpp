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

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include <fmt/format.h>

#include "../temp_directory.h"
#include "error/prelude.h"
#include "io/bytes.h"
#include "io/serde.h"

TEST_CASE("Reading truncated bytes reports that the data ended early") {
    const auto bytes = io::serde::serialize(std::vector<uint32_t>{1, 2, 3});
    REQUIRE(bytes.has_value());

    const std::span<const uint8_t> truncated = std::span(bytes.value()).first(bytes.value().size() - 1);
    const auto result = io::serde::deserialize<std::vector<uint32_t>>(truncated);

    REQUIRE(!result.has_value());
    CHECK(result.error().holds<io::serde::Truncated>());
}

TEST_CASE("deserialize rejects bytes after the value", "[io][serde]") {
    const auto serialized = io::serde::serialize(std::vector<uint32_t>{1, 2, 3});
    REQUIRE(serialized.has_value());
    std::vector<uint8_t> bytes = serialized.value();
    bytes.push_back(0);

    const auto result = io::serde::deserialize<std::vector<uint32_t>>(bytes);

    REQUIRE(!result.has_value());
    CHECK(result.error().holds<io::serde::TrailingData>());
}

TEST_CASE("deserialize_prefix returns the value and the bytes after it", "[io][serde]") {
    const auto serialized = io::serde::serialize(std::vector<uint32_t>{1, 2, 3});
    REQUIRE(serialized.has_value());
    std::vector<uint8_t> bytes = serialized.value();
    bytes.insert(bytes.end(), {4, 5, 6});

    const auto result = io::serde::deserialize_prefix<std::vector<uint32_t>>(bytes);

    REQUIRE(result.has_value());
    CHECK(result.value().value == std::vector<uint32_t>{1, 2, 3});
    CHECK(std::ranges::equal(result.value().rest, std::array<uint8_t, 3>{4, 5, 6}));
}

TEST_CASE("deserialize rejects a sequence longer than the buffer limit", "[io][serde]") {
    // A little-endian 32-bit length prefix of 2^27 + 1 elements, so 8-byte elements need 8 bytes more than 2^30.
    const std::vector<uint8_t> bytes = {0x01, 0x00, 0x00, 0x08};

    const auto result = io::serde::deserialize<std::vector<uint64_t>>(bytes);

    REQUIRE(!result.has_value());
    CHECK(result.error().holds<io::BufferLimitExceeded>());
}

TEST_CASE("Reading a value from a missing file fails with a file error") {
    const TempDirectory directory("io_error_missing_value");
    const std::filesystem::path path = directory.path() / "missing.bin";
    const auto result = io::serde::read<std::vector<uint32_t>>(path);

    REQUIRE(!result.has_value());
    CHECK(result.error().holds<io::FileError>());
}

TEST_CASE("read rejects a file with truncated data") {
    const TempDirectory directory("io_error_truncated_value");
    const std::filesystem::path path = directory.path() / "truncated.bin";
    REQUIRE(io::bytes::write(io::bytes::vector{1}, path).has_value());

    const auto result = io::serde::read<std::vector<uint32_t>>(path);

    REQUIRE(!result.has_value());
    CHECK(result.error().holds<io::serde::DeserializeError>());
}

TEST_CASE("Writing and reading a value round-trips it") {
    const TempDirectory directory("io_error_value_round_trip");
    const std::filesystem::path path = directory.path() / "value.bin";
    const std::vector<uint32_t> value = {1, 2, 3};

    REQUIRE(io::serde::write(value, path).has_value());
    const auto result = io::serde::read<std::vector<uint32_t>>(path);

    REQUIRE(result.has_value());
    CHECK(result.value() == value);
}
