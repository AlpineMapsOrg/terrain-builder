#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"

#include <filesystem>
#include <system_error>
#include <utility>

#include "error/Error.h"
#include "io/serde.h"
#include "io/FileError.h"
#include "io/filesystem.h"

#include "frontier/Storage.h"
#include "frontier/layout.h"

TEST_CASE("BatchStorage::create starts without batches in a missing or an empty directory", "[storage][BatchStorage]") {
    const TempDirectory directory("batch_storage");
    const std::filesystem::path path = directory.path() / "storage";

    const bool exists_already = GENERATE(false, true);
    if (exists_already) {
        REQUIRE_VALUE(io::filesystem::create_directories(path));
    }

    const frontier::Storage storage = REQUIRE_VALUE(frontier::Storage::create(path));

    CHECK(storage.size() == 0);
    CHECK(storage.directory() == path);
}

TEST_CASE("BatchStorage::create fails in a directory that holds a storage, naming it", "[storage][BatchStorage]") {
    const TempDirectory directory("batch_storage");
    const std::filesystem::path path = directory.path() / "storage";

    {
        frontier::Storage earlier = REQUIRE_VALUE(frontier::Storage::create(path));
        REQUIRE_VALUE(earlier.write(frontier::Batch{}));
        REQUIRE_VALUE(earlier.save_index());
    }

    const alp::Error<io::FileError> error = REQUIRE_ERROR(frontier::Storage::create(path));

    CHECK(error.holds_code(std::errc::directory_not_empty));
}

TEST_CASE("BatchStorage::open gives back the batches a saved storage wrote", "[storage][BatchStorage]") {
    const TempDirectory directory("batch_storage");
    const std::filesystem::path path = directory.path() / "storage";

    const frontier::Batch first{.entries = {{.morton = 7, .header = {}, .ref = {.batch = 0, .node = 0}}}};
    const frontier::Batch second{.entries = {{.morton = 3, .header = {}, .ref = {.batch = 0, .node = 1}}}};
    {
        frontier::Storage written = REQUIRE_VALUE(frontier::Storage::create(path));
        REQUIRE_VALUE(written.write(first));
        REQUIRE_VALUE(written.write(second));
        REQUIRE_VALUE(written.save_index());
    }

    const frontier::Storage opened = REQUIRE_VALUE(frontier::Storage::open(path));

    REQUIRE(opened.size() == 2);
    CHECK(REQUIRE_VALUE(opened.read(0)) == first);
    CHECK(REQUIRE_VALUE(opened.read(1)) == second);
}

TEST_CASE("BatchStorage::open fails in a directory without an index", "[storage][BatchStorage]") {
    const TempDirectory directory("batch_storage");
    const std::filesystem::path path = directory.path() / "storage";

    const bool exists_already = GENERATE(false, true);
    if (exists_already) {
        REQUIRE_VALUE(io::filesystem::create_directories(path));
    }

    const io::serde::Error error = REQUIRE_ERROR(frontier::Storage::open(path));

    CHECK(error.holds_and<io::FileError>(&io::FileError::is_not_found));
}

TEST_CASE("BatchStorage::remove deletes its own directory and nothing else", "[storage][BatchStorage]") {
    const TempDirectory directory("batch_storage");

    // A sibling under the same parent, so removing the parent instead of the storage would be caught.
    const frontier::Batch kept_batch{.entries = {{.morton = 7, .header = {}, .ref = {.batch = 0, .node = 0}}}};
    frontier::Storage kept = REQUIRE_VALUE(frontier::Storage::create(directory.path() / "kept"));
    REQUIRE_VALUE(kept.write(kept_batch));
    REQUIRE_VALUE(kept.save_index());

    frontier::Storage removed = REQUIRE_VALUE(frontier::Storage::create(directory.path() / "removed"));
    REQUIRE_VALUE(removed.write(frontier::Batch{.entries = {{.morton = 3, .header = {}, .ref = {.batch = 0, .node = 1}}}}));
    REQUIRE_VALUE(removed.write(frontier::Batch{}));
    REQUIRE_VALUE(removed.save_index());

    REQUIRE_VALUE(std::move(removed).remove());

    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(directory.path() / "removed")));
    const frontier::Batch stored_batch = REQUIRE_VALUE(kept.read(0));
    CHECK(stored_batch == kept_batch);
}

TEST_CASE("BatchStorage::remove leaves nothing for the dropped storage to write back", "[storage][BatchStorage]") {
    const TempDirectory directory("batch_storage");

    {
        // The index is left unsaved, so a storage that still thought itself dirty would write it on destruction.
        frontier::Storage storage = REQUIRE_VALUE(frontier::Storage::create(directory.path() / "removed"));
        REQUIRE_VALUE(storage.write(frontier::Batch{}));

        REQUIRE_VALUE(std::move(storage).remove());
    }

    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(directory.path() / "removed")));
}

TEST_CASE("BatchStorage::remove accepts a storage that never wrote anything", "[storage][BatchStorage]") {
    const TempDirectory directory("batch_storage");
    frontier::Storage storage = REQUIRE_VALUE(frontier::Storage::create(directory.path() / "never_written"));

    REQUIRE_VALUE(std::move(storage).remove());

    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(directory.path() / "never_written")));
}
