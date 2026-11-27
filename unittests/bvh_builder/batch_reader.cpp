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
        REQUIRE(storage.write(frontier::Batch{.entries = entries}).has_value());
    }
    REQUIRE(storage.save_index().has_value());
}

// Takes entries until the reader is exhausted.
std::vector<frontier::Entry> take_all(frontier::BatchReader &reader) {
    std::vector<frontier::Entry> entries;
    while (true) {
        const auto entry_result = reader.next();
        REQUIRE(entry_result.has_value());
        if (!entry_result->has_value()) {
            return entries;
        }
        entries.push_back(entry_result->value());
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
    const auto after_end = reader.next();
    REQUIRE(after_end.has_value());
    CHECK_FALSE(after_end->has_value());
}

TEST_CASE("BatchReader::next finds nothing in a storage without entries", "[frontier][BatchReader]") {
    const TempDirectory directory("batch_reader");
    frontier::Storage storage = REQUIRE_VALUE(frontier::Storage::create(directory.path()));

    const std::vector<std::vector<frontier::Entry>> batches = GENERATE(
        std::vector<std::vector<frontier::Entry>>{},
        std::vector<std::vector<frontier::Entry>>{{}, {}});
    write_batches(storage, batches);

    frontier::BatchReader reader(storage);

    const auto entry = reader.next();
    REQUIRE(entry.has_value());
    CHECK_FALSE(entry->has_value());
}

TEST_CASE("BatchReader::next reports the first batch it cannot read", "[frontier][BatchReader]") {
    const TempDirectory directory("batch_reader");
    frontier::Storage storage = REQUIRE_VALUE(frontier::Storage::create(directory.path() / "storage"));

    const frontier::Entry a = make_entry(7, 0);
    const frontier::Entry b = make_entry(3, 1);
    const frontier::Entry c = make_entry(9, 2);
    write_batches(storage, {{a, b}, {c}});

    frontier::BatchReader reader(storage);

    const auto first = reader.next();
    REQUIRE(first.has_value());
    CHECK(first.value() == a);

    // The first batch is already read, so only the second one is lost.
    REQUIRE_VALUE(io::filesystem::remove_all(directory.path() / "storage"));

    const auto second = reader.next();
    REQUIRE(second.has_value());
    CHECK(second.value() == b);

    const auto third = reader.next();
    REQUIRE_FALSE(third.has_value());
    CHECK(third.error().holds_and<io::FileError>([](const io::FileError &error) { return error.is_not_found(); }));
}
