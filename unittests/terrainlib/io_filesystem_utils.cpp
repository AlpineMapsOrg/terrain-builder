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
#include <set>
#include <system_error>
#include <vector>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "error/prelude.h"
#include "io/FileError.h"
#include "io/bytes.h"
#include "io/filesystem.h"
#include "io/filesystem_utils.h"
#include "io/text.h"

namespace {

// Writes a one-byte file at `path`.
void write_file(const std::filesystem::path &path) {
    REQUIRE_VALUE(io::bytes::write(std::vector<uint8_t>{1}, path));
}

} // namespace

TEST_CASE("A directory is reported as empty until it holds a file", "[io][filesystem_utils]") {
    const TempDirectory directory("io_filesystem_empty");

    CHECK(REQUIRE_VALUE(io::filesystem::is_empty(directory.path())));
    CHECK(REQUIRE_VALUE(io::filesystem::is_empty_directory(directory.path())));
    CHECK(REQUIRE_VALUE(io::filesystem::is_missing_or_empty_directory(directory.path())));

    write_file(directory.path() / "file");

    CHECK(!REQUIRE_VALUE(io::filesystem::is_empty(directory.path())));
    CHECK(!REQUIRE_VALUE(io::filesystem::is_empty_directory(directory.path())));
    CHECK(!REQUIRE_VALUE(io::filesystem::is_missing_or_empty_directory(directory.path())));
}

TEST_CASE("A regular file is not reported as an empty directory", "[io][filesystem_utils]") {
    const TempDirectory directory("io_filesystem_empty_file");
    const std::filesystem::path file = directory.path() / "file";
    write_file(file);

    CHECK(!REQUIRE_VALUE(io::filesystem::is_empty_directory(file)));
    CHECK(!REQUIRE_VALUE(io::filesystem::is_missing_or_empty_directory(file)));
}

TEST_CASE("A missing path is reported as missing, not as an empty directory", "[io][filesystem_utils]") {
    const TempDirectory directory("io_filesystem_empty_missing");
    const std::filesystem::path missing = directory.path() / "missing";

    CHECK(!REQUIRE_VALUE(io::filesystem::is_empty_directory(missing)));
    CHECK(REQUIRE_VALUE(io::filesystem::is_missing_or_empty_directory(missing)));

    const auto error = REQUIRE_ERROR(io::filesystem::is_empty(missing));
    CHECK(error.holds_code(io::make_file_not_found(missing)));
}

TEST_CASE("Creating an empty directory creates its missing parents and accepts one that is already empty", "[io][filesystem_utils]") {
    const TempDirectory directory("io_filesystem_create_empty");
    const std::filesystem::path nested = directory.path() / "parent" / "nested";

    REQUIRE_VALUE(io::filesystem::create_empty_directory(nested));
    CHECK(REQUIRE_VALUE(io::filesystem::is_empty_directory(nested)));

    REQUIRE_VALUE(io::filesystem::create_empty_directory(nested));
    CHECK(REQUIRE_VALUE(io::filesystem::is_empty_directory(nested)));
}

TEST_CASE("Creating an empty directory fails where one holds something, naming it and leaving it alone", "[io][filesystem_utils]") {
    const TempDirectory directory("io_filesystem_create_empty_used");
    const std::filesystem::path used = directory.path() / "used";
    write_file(used / "file");

    const auto error = REQUIRE_ERROR(io::filesystem::create_empty_directory(used));

    CHECK(error.holds_code(io::FileError(std::make_error_code(std::errc::directory_not_empty), used)));
    CHECK(REQUIRE_VALUE(io::filesystem::is_regular_file(used / "file")));
}

TEST_CASE("Creating an empty directory fails where a regular file is in the way, naming it", "[io][filesystem_utils]") {
    const TempDirectory directory("io_filesystem_create_empty_file");
    const std::filesystem::path file = directory.path() / "file";
    write_file(file);

    const auto error = REQUIRE_ERROR(io::filesystem::create_empty_directory(file));

    CHECK(error.holds_and<io::FileError>([&](const io::FileError &e) { return e.path() == file; }));
}

TEST_CASE("Visiting files recursively visits every file in nested directories and no directory", "[io][filesystem_utils]") {
    const TempDirectory directory("io_filesystem_visit_all");
    const std::set<std::filesystem::path> files = {
        directory.path() / "a",
        directory.path() / "nested" / "b",
        directory.path() / "nested" / "deeper" / "c",
    };
    for (const std::filesystem::path &file : files) {
        write_file(file);
    }

    std::set<std::filesystem::path> visited;
    REQUIRE_VALUE(io::filesystem::visit_files_recursively(directory.path(), [&](const std::filesystem::path &path) {
        visited.insert(path);
        return true;
    }));

    CHECK(visited == files);
}

TEST_CASE("Visiting files recursively stops once the visitor returns false", "[io][filesystem_utils]") {
    const TempDirectory directory("io_filesystem_visit_stop");
    write_file(directory.path() / "a");
    write_file(directory.path() / "b");
    write_file(directory.path() / "c");

    uint32_t visit_count = 0;
    REQUIRE_VALUE(io::filesystem::visit_files_recursively(directory.path(), [&](const std::filesystem::path &) {
        visit_count++;
        return false;
    }));

    CHECK(visit_count == 1);
}

TEST_CASE("Visiting files below a missing path fails, naming the path", "[io][filesystem_utils]") {
    const TempDirectory directory("io_filesystem_visit_missing");
    const std::filesystem::path missing = directory.path() / "missing";

    const auto error = REQUIRE_ERROR(io::filesystem::visit_files_recursively(missing, [](const std::filesystem::path &) {
        return true;
    }));

    CHECK(error.holds_code(io::make_file_not_found(missing)));
}

TEST_CASE("create_parent_directories creates missing parents and accepts existing ones", "[io][filesystem_utils]") {
    const TempDirectory directory("io_filesystem_create_parent_directories");
    const std::filesystem::path path = directory.path() / "parent" / "nested" / "file";

    CHECK(REQUIRE_VALUE(io::filesystem::create_parent_directories(path)));
    CHECK(REQUIRE_VALUE(io::filesystem::is_directory(path.parent_path())));
    CHECK(!REQUIRE_VALUE(io::filesystem::exists(path)));
    CHECK(!REQUIRE_VALUE(io::filesystem::create_parent_directories(path)));
}

TEST_CASE("rename_no_replace moves the source to a free destination", "[io][filesystem_utils]") {
    const TempDirectory directory("io_filesystem_rename_no_replace");
    const std::filesystem::path source = directory.path() / "source";
    const std::filesystem::path destination = directory.path() / "destination";
    REQUIRE_VALUE(io::text::write("source", source));

    REQUIRE_VALUE(io::filesystem::rename_no_replace(source, destination));

    CHECK(!REQUIRE_VALUE(io::filesystem::exists(source)));
    CHECK(REQUIRE_VALUE(io::text::read(destination)) == "source");
}

TEST_CASE("rename_no_replace fails onto an existing destination, naming it and leaving both alone", "[io][filesystem_utils]") {
    const TempDirectory directory("io_filesystem_rename_no_replace_existing");
    const std::filesystem::path source = directory.path() / "source";
    const std::filesystem::path destination = directory.path() / "destination";

    SECTION("file onto a file") {
        REQUIRE_VALUE(io::text::write("source", source));
        REQUIRE_VALUE(io::text::write("destination", destination));

        const auto error = REQUIRE_ERROR(io::filesystem::rename_no_replace(source, destination));

        CHECK(error.holds_code(io::FileError(std::make_error_code(std::errc::file_exists), destination)));
        CHECK(REQUIRE_VALUE(io::text::read(source)) == "source");
        CHECK(REQUIRE_VALUE(io::text::read(destination)) == "destination");
    }

    SECTION("directory onto an empty directory") {
        REQUIRE_VALUE(io::text::write("source", source / "file"));
        REQUIRE_VALUE(io::filesystem::create_directory(destination));

        const auto error = REQUIRE_ERROR(io::filesystem::rename_no_replace(source, destination));

        CHECK(error.holds_code(io::FileError(std::make_error_code(std::errc::file_exists), destination)));
        CHECK(REQUIRE_VALUE(io::text::read(source / "file")) == "source");
        CHECK(REQUIRE_VALUE(io::filesystem::is_empty_directory(destination)));
    }
}

TEST_CASE("rename_no_replace fails as not found for a missing source or destination parent", "[io][filesystem_utils]") {
    const TempDirectory directory("io_filesystem_rename_no_replace_missing");
    const std::filesystem::path source = directory.path() / "source";
    std::filesystem::path destination = directory.path() / "destination";

    SECTION("missing source") {
    }

    SECTION("missing destination parent") {
        write_file(source);
        destination = directory.path() / "missing" / "destination";
    }

    const auto error = REQUIRE_ERROR(io::filesystem::rename_no_replace(source, destination));

    CHECK(error.holds_and<io::FileError>([](const io::FileError &e) { return e.is_not_found(); }));
}

TEST_CASE("sync_file accepts a file", "[io][filesystem_utils]") {
    const TempDirectory directory("io_filesystem_sync_file");
    const std::filesystem::path file = directory.path() / "file";
    write_file(file);

    REQUIRE_VALUE(io::filesystem::sync_file(file));
}

TEST_CASE("sync_file fails for a missing path, naming it", "[io][filesystem_utils]") {
    const TempDirectory directory("io_filesystem_sync_file_missing");
    const std::filesystem::path missing = directory.path() / "missing";

    const auto error = REQUIRE_ERROR(io::filesystem::sync_file(missing));

    CHECK(error.holds_code(io::make_file_not_found(missing)));
}

TEST_CASE("require_absent accepts a path that nothing is at", "[io][filesystem_utils]") {
    const TempDirectory directory("io_filesystem_require_absent_missing");

    REQUIRE_VALUE(io::filesystem::require_absent(directory.path() / "missing"));
}

TEST_CASE("require_absent fails for anything at the path, a broken symlink included, naming it", "[io][filesystem_utils]") {
    const TempDirectory directory("io_filesystem_require_absent_present");
    const std::filesystem::path path = directory.path() / "taken";
    SECTION("a file") {
        write_file(path);
    }
    SECTION("a directory") {
        REQUIRE_VALUE(io::filesystem::create_directory(path));
    }
    SECTION("a broken symlink") {
        REQUIRE_VALUE(io::filesystem::create_symlink(directory.path() / "nowhere", path));
    }

    const auto error = REQUIRE_ERROR(io::filesystem::require_absent(path));

    CHECK(error.holds_code(io::FileError(std::errc::file_exists, path)));
}

TEST_CASE("require_same_filesystem accepts an output whose closest existing ancestor shares the source's filesystem", "[io][filesystem_utils]") {
    const TempDirectory directory("io_filesystem_require_same_filesystem");
    const std::filesystem::path source = directory.path() / "source";
    write_file(source);

    REQUIRE_VALUE(io::filesystem::require_same_filesystem(source, directory.path() / "missing" / "output"));
}

TEST_CASE("require_same_filesystem fails for a missing source, naming it", "[io][filesystem_utils]") {
    const TempDirectory directory("io_filesystem_require_same_filesystem_missing");
    const std::filesystem::path source = directory.path() / "missing";

    const auto error = REQUIRE_ERROR(io::filesystem::require_same_filesystem(source, directory.path() / "output"));

    CHECK(error.holds_code(io::make_file_not_found(source)));
}
