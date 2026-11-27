#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "frontier_helpers.h"

#include <cstddef>
#include <vector>

#include "frontier/Storage.h"
#include "frontier/layout.h"
#include "sort/options.h"
#include "sort/sort_by_morton.h"

namespace {

constexpr size_t ENTRY_BYTES = sizeof(frontier::Entry);

}

TEST_CASE("sort_by_morton writes the entries of all batches in morton order", "[sort][sort_by_morton]") {
    const TempDirectory directory("sort_by_morton");

    const frontier::Entry a{.morton = 7, .header = {}, .ref = {.batch = 0, .node = 0}};
    const frontier::Entry b{.morton = 3, .header = {}, .ref = {.batch = 0, .node = 1}};
    const frontier::Entry c{.morton = 9, .header = {}, .ref = {.batch = 0, .node = 2}};
    const frontier::Entry d{.morton = 6, .header = {}, .ref = {.batch = 0, .node = 3}};
    const frontier::Entry e{.morton = 1, .header = {}, .ref = {.batch = 0, .node = 4}};
    const frontier::Entry f{.morton = 5, .header = {}, .ref = {.batch = 0, .node = 5}};
    const frontier::Entry g{.morton = 2, .header = {}, .ref = {.batch = 0, .node = 6}};
    const frontier::Storage input = make_storage(directory.path() / "input", {{a, b, c}, {}, {d, e}, {f, g}});

    // The budget is far smaller than the input in one case and far larger in the other.
    const size_t max_batch_bytes = 2 * ENTRY_BYTES;
    const size_t max_resident_bytes = GENERATE(2 * ENTRY_BYTES, 1000 * ENTRY_BYTES);

    frontier::Storage output = REQUIRE_VALUE(frontier::Storage::create(directory.path() / "output"));

    const sort::Options options{
        .max_batch_bytes = max_batch_bytes,
        .max_resident_bytes = max_resident_bytes,
    };
    REQUIRE_VALUE(sort::sort_by_morton(input, output, directory.path() / "scratch", options));

    CHECK(read_batches(output) == std::vector<std::vector<frontier::Entry>>{
        {e, g},
        {b, f},
        {d, a},
        {c},
    });
}
