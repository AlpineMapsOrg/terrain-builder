#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <filesystem>
#include <system_error>
#include <utility>
#include <vector>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "error/prelude.h"
#include "io/FileError.h"
#include "io/ScratchDirectory.h"
#include "io/bytes.h"
#include "io/filesystem.h"

namespace {

// Writes a one-byte file at `path`.
void write_dummy_file(const std::filesystem::path &path) {
    REQUIRE_VALUE(io::bytes::write(std::vector<uint8_t>{1}, path));
}

} // namespace

TEST_CASE("A scratch directory is created empty with its missing parents", "[io][ScratchDirectory]") {
    const TempDirectory directory("io_scratch_directory_create");
    const std::filesystem::path nested = directory.path() / "parent" / "nested";

    const io::ScratchDirectory scratch = REQUIRE_VALUE(io::create_scratch_directory(nested));

    CHECK(scratch.path() == nested);
    CHECK(REQUIRE_VALUE(io::filesystem::is_empty_directory(nested)));
}

TEST_CASE("A scratch directory is removed with everything below it when it goes away, and nothing else is", "[io][ScratchDirectory]") {
    const TempDirectory directory("io_scratch_directory_drop");
    const std::filesystem::path kept = directory.path() / "kept";
    write_dummy_file(kept);

    {
        const io::ScratchDirectory scratch = REQUIRE_VALUE(io::create_scratch_directory(directory.path() / "scratch"));
        write_dummy_file(scratch.path() / "file");
        write_dummy_file(scratch.path() / "nested" / "file");
    }

    CHECK(!REQUIRE_VALUE(io::filesystem::exists(directory.path() / "scratch")));
    CHECK(REQUIRE_VALUE(io::filesystem::is_regular_file(kept)));
}

TEST_CASE("Creating a scratch directory fails where one holds something, naming it and leaving it alone", "[io][ScratchDirectory]") {
    const TempDirectory directory("io_scratch_directory_used");
    const std::filesystem::path used = directory.path() / "used";
    write_dummy_file(used / "file");

    const alp::Error<io::FileError> error = REQUIRE_ERROR(io::create_scratch_directory(used));

    CHECK(error.holds_code(std::errc::directory_not_empty));
    CHECK(REQUIRE_VALUE(io::filesystem::is_regular_file(used / "file")));
}

TEST_CASE("ScratchDirectory::remove removes the directory at once", "[io][ScratchDirectory]") {
    const TempDirectory directory("io_scratch_directory_remove");
    io::ScratchDirectory scratch = REQUIRE_VALUE(io::create_scratch_directory(directory.path() / "scratch"));
    write_dummy_file(scratch.path() / "file");

    REQUIRE_VALUE(std::move(scratch).remove());

    CHECK(!REQUIRE_VALUE(io::filesystem::exists(directory.path() / "scratch")));
}
