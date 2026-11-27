#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "frontier_helpers.h"

#include <cstddef>
#include <filesystem>
#include <vector>

#include "io/FileError.h"

#include "frontier/Storage.h"
#include "frontier/layout.h"
#include "sort/split_into_sorted_runs.h"

namespace {

constexpr size_t ENTRY_BYTES = sizeof(frontier::Entry);

}

TEST_CASE("split_into_sorted_runs sorts each run and cuts runs at the size limit", "[sort][split_into_sorted_runs]") {
    const TempDirectory directory("split_into_sorted_runs");

    // The first batch is larger than a run, and the second run takes its rest together with the following batches.
    const frontier::Entry a{.morton = 7, .header = {}, .ref = {.batch = 0, .node = 0}};
    const frontier::Entry b{.morton = 3, .header = {}, .ref = {.batch = 0, .node = 1}};
    const frontier::Entry c{.morton = 9, .header = {}, .ref = {.batch = 0, .node = 2}};
    const frontier::Entry d{.morton = 6, .header = {}, .ref = {.batch = 0, .node = 3}};
    const frontier::Entry e{.morton = 1, .header = {}, .ref = {.batch = 0, .node = 4}};
    const frontier::Entry f{.morton = 5, .header = {}, .ref = {.batch = 0, .node = 5}};
    const frontier::Entry g{.morton = 2, .header = {}, .ref = {.batch = 0, .node = 6}};
    const frontier::Storage input = make_storage(directory.path() / "input", {{a, b, c, d}, {}, {e}, {f, g}});

    // A run ends once it holds the limit: two entries stay just below the first limit and three meet the second exactly, so both give runs of three.
    const size_t max_run_bytes = GENERATE(2 * ENTRY_BYTES + 1, 3 * ENTRY_BYTES);
    const size_t max_batch_bytes = 2 * ENTRY_BYTES;

    const std::vector<frontier::Storage> runs = REQUIRE_VALUE(sort::split_into_sorted_runs(input, directory.path() / "runs", max_run_bytes, max_batch_bytes));

    std::vector<std::vector<std::vector<frontier::Entry>>> batches_per_run;
    for (const frontier::Storage &run : runs) {
        batches_per_run.push_back(read_batches(run));
    }

    // Each run is sorted on its own, so sorting across runs would be caught.
    CHECK(batches_per_run == std::vector<std::vector<std::vector<frontier::Entry>>>{
        {{b, a}, {c}},
        {{e, f}, {d}},
        {{g}},
    });
}

TEST_CASE("split_into_sorted_runs makes no runs from a storage without entries", "[sort][split_into_sorted_runs]") {
    const TempDirectory directory("split_into_sorted_runs");

    const std::vector<std::vector<frontier::Entry>> batches = GENERATE(
        std::vector<std::vector<frontier::Entry>>{},
        std::vector<std::vector<frontier::Entry>>{{}, {}});
    const frontier::Storage input = make_storage(directory.path() / "input", batches);

    const std::vector<frontier::Storage> runs = REQUIRE_VALUE(sort::split_into_sorted_runs(input, directory.path() / "runs", 3 * ENTRY_BYTES, 2 * ENTRY_BYTES));

    CHECK(runs.empty());
}

TEST_CASE("split_into_sorted_runs reports an input batch it cannot read", "[sort][split_into_sorted_runs]") {
    const TempDirectory directory("split_into_sorted_runs");

    const frontier::Entry a{.morton = 7, .header = {}, .ref = {.batch = 0, .node = 0}};
    const frontier::Entry b{.morton = 3, .header = {}, .ref = {.batch = 0, .node = 1}};
    const frontier::Entry c{.morton = 9, .header = {}, .ref = {.batch = 0, .node = 2}};
    const frontier::Storage input = make_storage(directory.path() / "input", {{a, b}, {c}});

    std::filesystem::remove_all(directory.path() / "input");

    const io::serde::Error error = REQUIRE_ERROR(sort::split_into_sorted_runs(input, directory.path() / "runs", 3 * ENTRY_BYTES, 2 * ENTRY_BYTES));
    CHECK(error.holds_and<io::FileError>(&io::FileError::is_not_found));
}
