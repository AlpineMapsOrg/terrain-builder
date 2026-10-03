#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <filesystem>
#include <set>
#include <vector>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "error/prelude.h"
#include "io/FileError.h"
#include "io/bytes.h"
#include "io/filesystem.h"

namespace {

// Writes a one-byte file at `path`.
void write_file(const std::filesystem::path &path) {
    REQUIRE_VALUE(io::write_bytes_to_path(std::vector<uint8_t>{1}, path));
}

} // namespace

TEST_CASE("A directory is reported as an existing directory only", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_directory");

    CHECK(REQUIRE_VALUE(io::filesystem::exists(directory.path())));
    CHECK(REQUIRE_VALUE(io::filesystem::is_directory(directory.path())));
    CHECK(!REQUIRE_VALUE(io::filesystem::is_regular_file(directory.path())));
    CHECK(!REQUIRE_VALUE(io::filesystem::is_symlink(directory.path())));
}

TEST_CASE("A regular file is reported as an existing regular file only", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_regular_file");
    const std::filesystem::path file = directory.path() / "file";
    write_file(file);

    CHECK(REQUIRE_VALUE(io::filesystem::exists(file)));
    CHECK(!REQUIRE_VALUE(io::filesystem::is_directory(file)));
    CHECK(REQUIRE_VALUE(io::filesystem::is_regular_file(file)));
    CHECK(!REQUIRE_VALUE(io::filesystem::is_symlink(file)));
}

TEST_CASE("A symlink to a regular file is reported as a symlink and, followed, as a regular file", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_symlink");
    const std::filesystem::path file = directory.path() / "file";
    const std::filesystem::path link = directory.path() / "link";
    write_file(file);
    std::filesystem::create_symlink(file.filename(), link);

    CHECK(REQUIRE_VALUE(io::filesystem::exists(link)));
    CHECK(!REQUIRE_VALUE(io::filesystem::is_directory(link)));
    CHECK(REQUIRE_VALUE(io::filesystem::is_regular_file(link)));
    CHECK(REQUIRE_VALUE(io::filesystem::is_symlink(link)));
}

TEST_CASE("A broken symlink is reported as a symlink to nothing", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_broken_symlink");
    const std::filesystem::path link = directory.path() / "link";
    std::filesystem::create_symlink("missing", link);

    CHECK(!REQUIRE_VALUE(io::filesystem::exists(link)));
    CHECK(!REQUIRE_VALUE(io::filesystem::is_directory(link)));
    CHECK(!REQUIRE_VALUE(io::filesystem::is_regular_file(link)));
    CHECK(REQUIRE_VALUE(io::filesystem::is_symlink(link)));
}

TEST_CASE("A missing path is reported as nothing", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_missing");
    const std::filesystem::path missing = directory.path() / "missing";

    CHECK(!REQUIRE_VALUE(io::filesystem::exists(missing)));
    CHECK(!REQUIRE_VALUE(io::filesystem::is_directory(missing)));
    CHECK(!REQUIRE_VALUE(io::filesystem::is_regular_file(missing)));
    CHECK(!REQUIRE_VALUE(io::filesystem::is_symlink(missing)));
}

TEST_CASE("A directory is reported as empty until it holds a file", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_empty");

    CHECK(REQUIRE_VALUE(io::filesystem::is_empty(directory.path())));
    CHECK(REQUIRE_VALUE(io::filesystem::is_empty_directory(directory.path())));
    CHECK(REQUIRE_VALUE(io::filesystem::is_missing_or_empty_directory(directory.path())));

    write_file(directory.path() / "file");

    CHECK(!REQUIRE_VALUE(io::filesystem::is_empty(directory.path())));
    CHECK(!REQUIRE_VALUE(io::filesystem::is_empty_directory(directory.path())));
    CHECK(!REQUIRE_VALUE(io::filesystem::is_missing_or_empty_directory(directory.path())));
}

TEST_CASE("A regular file is not reported as an empty directory", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_empty_file");
    const std::filesystem::path file = directory.path() / "file";
    write_file(file);

    CHECK(!REQUIRE_VALUE(io::filesystem::is_empty_directory(file)));
    CHECK(!REQUIRE_VALUE(io::filesystem::is_missing_or_empty_directory(file)));
}

TEST_CASE("A missing path is reported as missing, not as an empty directory", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_empty_missing");
    const std::filesystem::path missing = directory.path() / "missing";

    CHECK(!REQUIRE_VALUE(io::filesystem::is_empty_directory(missing)));
    CHECK(REQUIRE_VALUE(io::filesystem::is_missing_or_empty_directory(missing)));

    const alp::Error<io::FileError> error = REQUIRE_ERROR(io::filesystem::is_empty(missing));
    CHECK(error.holds(io::make_file_not_found(missing)));
}

TEST_CASE("Visiting files recursively visits every file in nested directories and no directory", "[io][filesystem]") {
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

TEST_CASE("Visiting files recursively stops once the visitor returns false", "[io][filesystem]") {
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

TEST_CASE("Visiting files below a missing path fails, naming the path", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_visit_missing");
    const std::filesystem::path missing = directory.path() / "missing";

    const alp::Error<io::FileError> error = REQUIRE_ERROR(io::filesystem::visit_files_recursively(missing, [](const std::filesystem::path &) {
        return true;
    }));

    CHECK(error.holds(io::make_file_not_found(missing)));
}

TEST_CASE("create_directory creates a directory and reports that it was created", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_create_directory");
    const std::filesystem::path created = directory.path() / "created";

    CHECK(REQUIRE_VALUE(io::filesystem::create_directory(created)));
    CHECK(REQUIRE_VALUE(io::filesystem::is_directory(created)));
}

TEST_CASE("create_directory reports that an existing directory was not created", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_create_directory_existing");

    CHECK(!REQUIRE_VALUE(io::filesystem::create_directory(directory.path())));
}

TEST_CASE("create_directory fails below a missing parent", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_create_directory_missing_parent");

    REQUIRE_ERROR(io::filesystem::create_directory(directory.path() / "missing" / "created"));
}

TEST_CASE("remove deletes the file at a path and reports that there was one", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_remove");
    const std::filesystem::path file = directory.path() / "file";
    write_file(file);

    CHECK(REQUIRE_VALUE(io::filesystem::remove(file)));
    CHECK(!std::filesystem::exists(file));
}

TEST_CASE("remove reports that there was nothing to remove at a missing path", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_remove_missing");
    const std::filesystem::path missing = directory.path() / "missing";

    CHECK(!REQUIRE_VALUE(io::filesystem::remove(missing)));
}

TEST_CASE("remove fails on a non-empty directory", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_remove_non_empty");
    const std::filesystem::path non_empty = directory.path() / "non_empty";
    write_file(non_empty / "file");

    REQUIRE_ERROR(io::filesystem::remove(non_empty));
}

TEST_CASE("remove_all deletes a directory with everything in it and reports how many entries it removed", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_remove_all");
    const std::filesystem::path non_empty = directory.path() / "non_empty";
    write_file(non_empty / "file");

    CHECK(REQUIRE_VALUE(io::filesystem::remove_all(non_empty)) == 2);
    CHECK(!std::filesystem::exists(non_empty));
}

TEST_CASE("remove_all reports that there was nothing to remove at a missing path", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_remove_all_missing");
    const std::filesystem::path missing = directory.path() / "missing";

    CHECK(REQUIRE_VALUE(io::filesystem::remove_all(missing)) == 0);
}

TEST_CASE("create_hard_link makes the target another name for the source's file", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_hard_link");
    const std::filesystem::path source = directory.path() / "source";
    const std::filesystem::path target = directory.path() / "target";
    write_file(source);

    REQUIRE_VALUE(io::filesystem::create_hard_link(source, target));

    CHECK(REQUIRE_VALUE(io::filesystem::equivalent(source, target)));
}

TEST_CASE("create_hard_link fails for a missing source", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_hard_link_missing_source");

    REQUIRE_ERROR(io::filesystem::create_hard_link(directory.path() / "missing", directory.path() / "target"));
}

TEST_CASE("create_hard_link fails onto an existing target", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_hard_link_existing_target");
    const std::filesystem::path source = directory.path() / "source";
    const std::filesystem::path target = directory.path() / "target";
    write_file(source);
    write_file(target);

    REQUIRE_ERROR(io::filesystem::create_hard_link(source, target));
}

TEST_CASE("equivalent reports two separate files as different files", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_equivalent_separate");
    const std::filesystem::path first = directory.path() / "first";
    const std::filesystem::path second = directory.path() / "second";
    write_file(first);
    write_file(second);

    CHECK(!REQUIRE_VALUE(io::filesystem::equivalent(first, second)));
}

TEST_CASE("equivalent fails for a missing path", "[io][filesystem]") {
    const TempDirectory directory("io_filesystem_equivalent_missing");
    const std::filesystem::path file = directory.path() / "file";
    write_file(file);

    REQUIRE_ERROR(io::filesystem::equivalent(file, directory.path() / "missing"));
}
