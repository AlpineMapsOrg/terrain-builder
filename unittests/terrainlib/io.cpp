#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include <fmt/format.h>

#include "../temp_directory.h"
#include "error/prelude.h"
#include "io/Error.h"
#include "io/bytes.h"
#include "io/serialize.h"

TEST_CASE("Writing and reading a file round-trips its bytes") {
    const TempDirectory directory("io_error_round_trip");
    const std::filesystem::path path = directory.path() / "bytes.bin";
    const std::vector<uint8_t> bytes = {1, 2, 3};

    REQUIRE(io::write_bytes_to_path(bytes, path).has_value());
    const alp::Result<std::vector<uint8_t>, io::FileError> result = io::read_bytes_from_path(path);

    REQUIRE(result.has_value());
    CHECK(result.value() == bytes);
}

TEST_CASE("Reading a missing file reports that it does not exist, naming the path") {
    const TempDirectory directory("io_error_missing");
    const std::filesystem::path path = directory.path() / "missing.bin";
    const alp::Result<std::vector<uint8_t>, io::FileError> result = io::read_bytes_from_path(path);

    REQUIRE(!result.has_value());
    CHECK(result.error().holds(io::make_file_not_found(path)));
}

TEST_CASE("Reading a directory fails") {
    const TempDirectory directory("io_error_directory");

    CHECK(!io::read_bytes_from_path(directory.path()).has_value());
}

TEST_CASE("Writing below a file fails with a file error, naming the directory it could not create") {
    const TempDirectory directory("io_error_below_file");
    const std::filesystem::path blocker = directory.path() / "blocker";
    REQUIRE(io::write_bytes_to_path(std::vector<uint8_t>{1}, blocker).has_value());

    const std::filesystem::path path = blocker / "child.bin";
    const alp::Result<void, io::FileError> result = io::write_bytes_to_path(std::vector<uint8_t>{1}, path);

    REQUIRE(!result.has_value());
    CHECK(result.error().code().path() == blocker);
}

TEST_CASE("Reading truncated bytes reports that the data ended early") {
    const alp::Result<std::vector<uint8_t>, io::SerdeError> bytes = io::write_to_bytes(std::vector<uint32_t>{1, 2, 3});
    REQUIRE(bytes.has_value());

    const std::span<const uint8_t> truncated = std::span(bytes.value()).first(bytes.value().size() - 1);
    const alp::Result<std::vector<uint32_t>, io::SerdeError> result = io::read_from_bytes<std::vector<uint32_t>>(truncated);

    REQUIRE(!result.has_value());
    CHECK(result.error().holds(io::SerdeError(std::errc::result_out_of_range)));
}

TEST_CASE("Reading a value from a missing file fails with a file error") {
    const TempDirectory directory("io_error_missing_value");
    const std::filesystem::path path = directory.path() / "missing.bin";
    const io::Result<std::vector<uint32_t>> result = io::read_from_path<std::vector<uint32_t>>(path);

    REQUIRE(!result.has_value());
    CHECK(result.error().holds<io::FileError>());
}

TEST_CASE("Reading a value from a file with truncated data fails with a serde error") {
    const TempDirectory directory("io_error_truncated_value");
    const std::filesystem::path path = directory.path() / "truncated.bin";
    REQUIRE(io::write_bytes_to_path(std::vector<uint8_t>{1}, path).has_value());

    const io::Result<std::vector<uint32_t>> result = io::read_from_path<std::vector<uint32_t>>(path);

    REQUIRE(!result.has_value());
    CHECK(result.error().holds<io::SerdeError>());
}

TEST_CASE("Writing and reading a value round-trips it") {
    const TempDirectory directory("io_error_value_round_trip");
    const std::filesystem::path path = directory.path() / "value.bin";
    const std::vector<uint32_t> value = {1, 2, 3};

    REQUIRE(io::write_to_path(value, path).has_value());
    const io::Result<std::vector<uint32_t>> result = io::read_from_path<std::vector<uint32_t>>(path);

    REQUIRE(result.has_value());
    CHECK(result.value() == value);
}
