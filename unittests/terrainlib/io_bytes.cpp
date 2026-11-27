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

#include <cstdint>
#include <filesystem>
#include <system_error>
#include <vector>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "error/prelude.h"
#include "io/FileError.h"
#include "io/WriteMode.h"
#include "io/bytes.h"
#include "io/filesystem.h"

TEST_CASE("read returns the bytes write wrote", "[io][bytes]") {
    const TempDirectory directory("io_byte_round_trip");
    const std::filesystem::path path = directory.path() / "file";

    REQUIRE_VALUE(io::bytes::write(io::bytes::vector{1, 2, 3}, path));

    CHECK(REQUIRE_VALUE(io::bytes::read(path)) == io::bytes::vector{1, 2, 3});
}

TEST_CASE("write creates the missing parent directories", "[io][bytes]") {
    const TempDirectory directory("io_byte_parents");
    const std::filesystem::path path = directory.path() / "a" / "b" / "file";

    REQUIRE_VALUE(io::bytes::write(io::bytes::vector{1}, path));

    CHECK(REQUIRE_VALUE(io::filesystem::is_regular_file(path)));
}

TEST_CASE("write without make_dirs fails for a missing parent, naming the file", "[io][bytes]") {
    const TempDirectory directory("io_byte_no_parents");
    const std::filesystem::path path = directory.path() / "missing" / "file";

    const auto error = REQUIRE_ERROR(io::bytes::write(io::bytes::vector{1}, path, {.make_dirs = false}));

    CHECK(error.holds_and<io::FileError>([&](const io::FileError &e) { return e.path() == path; }));
}

TEST_CASE("write with CreateNew refuses an existing file", "[io][bytes]") {
    const TempDirectory directory("io_byte_create_new");
    const std::filesystem::path path = directory.path() / "file";
    REQUIRE_VALUE(io::bytes::write(io::bytes::vector{1, 2, 3}, path));

    const auto result = io::bytes::write(io::bytes::vector{4}, path, {.mode = io::WriteMode::CreateNew});

    SECTION("failing, naming the file") {
        const auto error = REQUIRE_ERROR(result);
        CHECK(error.holds_code(io::FileError(std::make_error_code(std::errc::file_exists), path)));
    }
    SECTION("leaving its bytes unchanged") {
        CHECK(REQUIRE_VALUE(io::bytes::read(path)) == io::bytes::vector{1, 2, 3});
    }
}

TEST_CASE("write with Overwrite replaces all bytes of a longer file", "[io][bytes]") {
    const TempDirectory directory("io_byte_overwrite");
    const std::filesystem::path path = directory.path() / "file";
    REQUIRE_VALUE(io::bytes::write(io::bytes::vector{1, 2, 3}, path));

    REQUIRE_VALUE(io::bytes::write(io::bytes::vector{4}, path, {.mode = io::WriteMode::Overwrite}));

    CHECK(REQUIRE_VALUE(io::bytes::read(path)) == io::bytes::vector{4});
}

TEST_CASE("write with Replace replaces the bytes of an existing file", "[io][bytes]") {
    const TempDirectory directory("io_byte_replace");
    const std::filesystem::path path = directory.path() / "file";
    const std::filesystem::path link = directory.path() / "link";
    const std::filesystem::path temporary_path = directory.path() / "file.tmp";
    REQUIRE_VALUE(io::bytes::write(io::bytes::vector{1, 2, 3}, path));
    REQUIRE_VALUE(io::filesystem::create_hard_link(path, link));
    // A temporary file left behind by a crash is truncated.
    REQUIRE_VALUE(io::bytes::write(io::bytes::vector{6, 7, 8, 9}, temporary_path));

    REQUIRE_VALUE(io::bytes::write(io::bytes::vector{4}, path, {.mode = io::WriteMode::Replace}));

    SECTION("at its path") {
        CHECK(REQUIRE_VALUE(io::bytes::read(path)) == io::bytes::vector{4});
    }
    SECTION("leaving hard links to the old file alone") {
        CHECK(REQUIRE_VALUE(io::bytes::read(link)) == io::bytes::vector{1, 2, 3});
    }
    SECTION("removing the temporary file") {
        CHECK(!REQUIRE_VALUE(io::filesystem::exists(temporary_path)));
    }
}

TEST_CASE("write fails where a file blocks a parent directory", "[io][bytes]") {
    const TempDirectory directory("io_byte_blocked_parent");
    const std::filesystem::path blocker = directory.path() / "blocker";
    REQUIRE_VALUE(io::bytes::write(io::bytes::vector{1}, blocker));

    REQUIRE_ERROR(io::bytes::write(io::bytes::vector{1}, blocker / "file", {.mode = io::WriteMode::Overwrite}));
}

TEST_CASE("write fails on a full device, naming it", "[io][bytes]") {
#ifdef __linux__
    const auto error = REQUIRE_ERROR(io::bytes::write(io::bytes::vector{1}, "/dev/full", {.mode = io::WriteMode::Overwrite}));

    CHECK(error.holds_and<io::FileError>([](const io::FileError &e) { return e.path() == "/dev/full"; }));
#endif
}

TEST_CASE("read fails as not found for a missing file, naming it", "[io][bytes]") {
    const TempDirectory directory("io_byte_read_missing");
    const std::filesystem::path path = directory.path() / "missing";

    const io::bytes::ReadError error = REQUIRE_ERROR(io::bytes::read(path));

    CHECK(error.holds_code(io::make_file_not_found(path)));
}

TEST_CASE("read fails for a directory", "[io][bytes]") {
    const TempDirectory directory("io_byte_read_directory");

    CHECK(!io::bytes::read(directory.path()).has_value());
}

TEST_CASE("read_prefix reads at most max_size bytes", "[io][bytes]") {
    const TempDirectory directory("io_byte_read_prefix");
    const std::filesystem::path path = directory.path() / "file";
    REQUIRE_VALUE(io::bytes::write(io::bytes::vector{1, 2, 3}, path));

    SECTION("of a longer file") {
        CHECK(REQUIRE_VALUE(io::bytes::read_prefix(path, 2)) == io::bytes::vector{1, 2});
    }
    SECTION("of a shorter file") {
        CHECK(REQUIRE_VALUE(io::bytes::read_prefix(path, 5)) == io::bytes::vector{1, 2, 3});
    }
}
