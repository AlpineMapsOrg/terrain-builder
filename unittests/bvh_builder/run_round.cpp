#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "frontier_helpers.h"
#include "node_header_helpers.h"

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "bvh/Storage.h"
#include "frontier/Storage.h"
#include "frontier/layout.h"
#include "gated_ploc/options.h"
#include "gated_ploc/run_round.h"
#include "morton/utils.h"

namespace {

// A cube of side `extent` at `x` along the x axis, at the given code.
frontier::Entry make_entry(const double extent, const double x, const morton::Code morton) {
    const glm::dvec3 min{x, 0.0, 0.0};
    return frontier::Entry{
        .morton = morton,
        .header = make_node_header({0.0, 0.0}, bvh::Bounds{min, min + extent}),
        .ref = bvh::Reference{},
    };
}

}

TEST_CASE("run_round writes the next run in morton order when the gate holds entries back", "[gated_ploc][run_round]") {
    // Regions of four, each admitting up to its second smallest extent, so the large 30 and 60 are held back.
    // The admitted boxes pair up as 10 with 20, 40 with 50 across the regions, and 70 with 80.
    const std::vector<frontier::Entry> entries{
        make_entry(1.0, 0.0, 10),
        make_entry(1.0, 1.0, 20),
        make_entry(5.0, 50.0, 30),
        make_entry(1.0, 100.0, 40),
        make_entry(1.0, 101.0, 50),
        make_entry(5.0, 150.0, 60),
        make_entry(1.0, 300.0, 70),
        make_entry(1.0, 301.0, 80),
    };
    const gated_ploc::Options options{
        .search_radius = 2,
        .admission = gated_ploc::AdmissionOptions{.region_size = 4, .quantile = 0.0, .tolerance = 1.0},
    };

    const TempDirectory directory("run_round");
    bvh::Storage output = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "output"));
    const frontier::Storage input = make_storage(directory.path() / "input", {entries});
    frontier::Storage next = REQUIRE_VALUE(frontier::Storage::create(directory.path() / "next"));

    const uint64_t remaining = REQUIRE_VALUE(gated_ploc::detail::run_round(input, next, output, options));
    REQUIRE_VALUE(output.save_index());
    REQUIRE_VALUE(next.save_index());

    std::vector<morton::Code> codes;
    for (const std::vector<frontier::Entry> &batch : read_batches(next)) {
        for (const frontier::Entry &entry : batch) {
            codes.push_back(entry.morton);
        }
    }
    CHECK(remaining == 5);
    CHECK(codes == std::vector<morton::Code>{10, 30, 40, 60, 70});
}
