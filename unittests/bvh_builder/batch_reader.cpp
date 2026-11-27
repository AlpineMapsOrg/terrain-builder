#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"

#include <filesystem>
#include <optional>
#include <vector>

#include "frontier/BatchReader.h"
#include "frontier/Storage.h"
#include "frontier/layout.h"
#include "io/filesystem.h"

namespace {

frontier::Entry make_entry(const morton::Code &code, const bvh::NodeIndex node) {
    return frontier::Entry{
        .morton = code,
        .header = {},
        .ref = {.batch = 0, .node = node},
    };
}

void write_batches(frontier::Storage &storage, const std::vector<std::vector<frontier::Entry>> &batches) {
    for (const std::vector<frontier::Entry> &entries : batches) {
        REQUIRE_VALUE(storage.write(frontier::Batch{.entries = entries}));
    }
    REQUIRE_VALUE(storage.save_index());
}

// Takes entries until the reader is exhausted.
std::vector<frontier::Entry> take_all(frontier::BatchReader &reader) {
    std::vector<frontier::Entry> entries;
    while (true) {
        const std::optional<frontier::Entry> entry_opt = REQUIRE_VALUE(reader.next());
        if (!entry_opt.has_value()) {
            return entries;
        }
        entries.push_back(entry_opt.value());
    }
}

}

TEST_CASE("BatchReader::next takes the entries of every batch in stored order", "[frontier][BatchReader]") {
    const TempDirectory directory("batch_reader");
    frontier::Storage storage = REQUIRE_VALUE(frontier::Storage::create(directory.path()));

    // Codes out of order, so a reader that sorted would be caught, and empty batches between and after the full ones.
    const frontier::Entry a = make_entry(7, 0);
    const frontier::Entry b = make_entry(3, 1);
    const frontier::Entry c = make_entry(9, 2);
    const frontier::Entry d = make_entry(1, 3);
    const frontier::Entry e = make_entry(4, 4);
    write_batches(storage, {{a, b}, {}, {c}, {d, e}, {}});

    frontier::BatchReader reader(storage);

    CHECK(take_all(reader) == std::vector<frontier::Entry>{a, b, c, d, e});

    // An exhausted reader stays exhausted.
    CHECK(REQUIRE_VALUE(reader.next()) == std::nullopt);
}

TEST_CASE("BatchReader::next finds nothing in a storage without entries", "[frontier][BatchReader]") {
    const TempDirectory directory("batch_reader");
    frontier::Storage storage = REQUIRE_VALUE(frontier::Storage::create(directory.path()));

    const std::vector<std::vector<frontier::Entry>> batches = GENERATE(
        std::vector<std::vector<frontier::Entry>>{},
        std::vector<std::vector<frontier::Entry>>{{}, {}});
    write_batches(storage, batches);

    frontier::BatchReader reader(storage);

    CHECK(REQUIRE_VALUE(reader.next()) == std::nullopt);
}

TEST_CASE("BatchReader::next reports the first batch it cannot read", "[frontier][BatchReader]") {
    const TempDirectory directory("batch_reader");
    frontier::Storage storage = REQUIRE_VALUE(frontier::Storage::create(directory.path() / "storage"));

    const frontier::Entry a = make_entry(7, 0);
    const frontier::Entry b = make_entry(3, 1);
    const frontier::Entry c = make_entry(9, 2);
    write_batches(storage, {{a, b}, {c}});

    frontier::BatchReader reader(storage);

    CHECK(REQUIRE_VALUE(reader.next()) == a);

    // The first batch is already read, so only the second one is lost.
    REQUIRE_VALUE(io::filesystem::remove_all(directory.path() / "storage"));

    CHECK(REQUIRE_VALUE(reader.next()) == b);

    const alp::Error<io::Error> error = REQUIRE_ERROR(reader.next());
    CHECK(error.holds_and<io::FileError>(&io::FileError::is_not_found));
}
