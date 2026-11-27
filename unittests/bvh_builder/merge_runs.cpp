#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "frontier_helpers.h"

#include <cstddef>
#include <filesystem>
#include <vector>

#include <fmt/format.h>

#include "io/serde.h"
#include "io/FileError.h"

#include "frontier/Storage.h"
#include "frontier/layout.h"
#include "sort/merge_runs.h"

namespace {

constexpr size_t ENTRY_BYTES = sizeof(frontier::Entry);

}

TEST_CASE("merge_runs writes the entries of all runs in morton order", "[sort][merge_runs]") {
    const TempDirectory directory("merge_runs");

    // The codes interleave across the runs, the second run is used up long before the others, and the third holds an empty batch.
    const frontier::Entry a{.morton = 1, .header = {}, .ref = {.batch = 0, .node = 0}};
    const frontier::Entry b{.morton = 4, .header = {}, .ref = {.batch = 0, .node = 1}};
    const frontier::Entry c{.morton = 9, .header = {}, .ref = {.batch = 0, .node = 2}};
    const frontier::Entry d{.morton = 2, .header = {}, .ref = {.batch = 0, .node = 3}};
    const frontier::Entry e{.morton = 3, .header = {}, .ref = {.batch = 0, .node = 4}};
    const frontier::Entry f{.morton = 5, .header = {}, .ref = {.batch = 0, .node = 5}};
    const frontier::Entry g{.morton = 6, .header = {}, .ref = {.batch = 0, .node = 6}};
    const frontier::Entry h{.morton = 8, .header = {}, .ref = {.batch = 0, .node = 7}};

    std::vector<frontier::Storage> runs;
    runs.push_back(make_storage(directory.path() / "run-0", {{a, b}, {c}}));
    runs.push_back(make_storage(directory.path() / "run-1", {{d}}));
    runs.push_back(make_storage(directory.path() / "run-2", {{e, f}, {}, {g, h}}));

    frontier::Storage output = REQUIRE_VALUE(frontier::Storage::create(directory.path() / "output"));

    REQUIRE_VALUE(sort::merge_runs(runs, output, 3 * ENTRY_BYTES));

    CHECK(read_batches(output) == std::vector<std::vector<frontier::Entry>>{
        {a, d, e},
        {b, f, g},
        {h, c},
    });
}

TEST_CASE("merge_runs writes nothing without entries", "[sort][merge_runs]") {
    const TempDirectory directory("merge_runs");

    // No runs at all, and runs that hold no batch or only an empty one.
    const std::vector<std::vector<std::vector<frontier::Entry>>> batches_per_run = GENERATE(
        std::vector<std::vector<std::vector<frontier::Entry>>>{},
        std::vector<std::vector<std::vector<frontier::Entry>>>{{}, {{}}});

    std::vector<frontier::Storage> runs;
    for (const std::vector<std::vector<frontier::Entry>> &batches : batches_per_run) {
        runs.push_back(make_storage(directory.path() / fmt::format("run-{}", runs.size()), batches));
    }

    frontier::Storage output = REQUIRE_VALUE(frontier::Storage::create(directory.path() / "output"));

    REQUIRE_VALUE(sort::merge_runs(runs, output, 3 * ENTRY_BYTES));

    CHECK(read_batches(output).empty());
}

TEST_CASE("merge_runs reports a run it cannot read", "[sort][merge_runs]") {
    const TempDirectory directory("merge_runs");

    const frontier::Entry a{.morton = 1, .header = {}, .ref = {.batch = 0, .node = 0}};
    const frontier::Entry b{.morton = 4, .header = {}, .ref = {.batch = 0, .node = 1}};
    const frontier::Entry c{.morton = 2, .header = {}, .ref = {.batch = 0, .node = 2}};

    std::vector<frontier::Storage> runs;
    runs.push_back(make_storage(directory.path() / "run-0", {{a, b}}));
    runs.push_back(make_storage(directory.path() / "run-1", {{c}}));

    std::filesystem::remove_all(directory.path() / "run-1");

    frontier::Storage output = REQUIRE_VALUE(frontier::Storage::create(directory.path() / "output"));

    const io::serde::Error error = REQUIRE_ERROR(sort::merge_runs(runs, output, 3 * ENTRY_BYTES));
    CHECK(error.holds_and<io::FileError>([](const io::FileError &e) { return e.is_not_found(); }));
}
