#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "frontier_helpers.h"

#include <vector>

#include "Range.h"
#include "frontier/Storage.h"
#include "frontier/traverse.h"

namespace {

frontier::Entry make_entry(const uint32_t morton) {
    return frontier::Entry{
        .morton = morton::Code{morton},
        .header = {},
        .ref = {},
    };
}

std::vector<uint32_t> collect_mortons(
    const frontier::Storage &run, const AnyRange<uint32_t> batches = RangeFull{}) {
    std::vector<uint32_t> visited;
    REQUIRE_VALUE(frontier::traverse_entries(
        run, [&](const frontier::Entry &entry) { visited.push_back(static_cast<uint32_t>(entry.morton)); }, batches));
    return visited;
}

}

TEST_CASE("traverse visits every entry in order", "[frontier][traverse]") {
    const TempDirectory directory("frontier_traverse");
    const frontier::Storage run = make_storage(directory.path(), {
        {make_entry(0), make_entry(1), make_entry(2)},
        {make_entry(3)},
        {make_entry(4), make_entry(5), make_entry(6), make_entry(7)},
    });

    CHECK(collect_mortons(run) == std::vector<uint32_t>{0, 1, 2, 3, 4, 5, 6, 7});
}

TEST_CASE("traverse over a batch range visits only those batches", "[frontier][traverse]") {
    const TempDirectory directory("frontier_traverse");
    const frontier::Storage run = make_storage(directory.path(), {
        {make_entry(0), make_entry(1), make_entry(2)},
        {make_entry(3)},
        {make_entry(4), make_entry(5), make_entry(6), make_entry(7)},
    });

    CHECK(collect_mortons(run, Range<uint32_t>{1, 3}) == std::vector<uint32_t>{3, 4, 5, 6, 7});
    CHECK(collect_mortons(run, Range<uint32_t>{0, 1}) == std::vector<uint32_t>{0, 1, 2});
    CHECK(collect_mortons(run, Range<uint32_t>{1, 1}).empty());
}

TEST_CASE("traverse handles an empty run and empty batches", "[frontier][traverse]") {
    const TempDirectory directory("frontier_traverse");

    const frontier::Storage empty = make_storage(directory.path() / "empty", {});
    CHECK(collect_mortons(empty).empty());

    const frontier::Storage sparse = make_storage(directory.path() / "sparse", {
        {},
        {make_entry(0), make_entry(1)},
        {},
    });
    CHECK(collect_mortons(sparse) == std::vector<uint32_t>{0, 1});
}
