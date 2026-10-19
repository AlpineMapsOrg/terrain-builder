#include <catch2/catch_test_macros.hpp>

#include <filesystem>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "io/text.h"

TEST_CASE("write_text_to_path and read_text_from_path round-trip text", "[io][text]") {
    const TempDirectory directory("io_text_round_trip");
    const std::filesystem::path path = directory.path() / "nested" / "file.txt";

    REQUIRE_VALUE(io::write_text_to_path("first line\nsecond line\n", path));

    CHECK(REQUIRE_VALUE(io::read_text_from_path(path)) == "first line\nsecond line\n");
}
