#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "frontier_helpers.h"
#include "node_header_helpers.h"

#include <cstddef>
#include <ranges>
#include <span>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "containers/PaddedSpan.h"
#include "range_utils.h"

#include "build/merge_cost.h"
#include "bvh/BatchWriter.h"
#include "bvh/Storage.h"
#include "frontier/layout.h"
#include "gated_ploc/ChunkMerger.h"
#include "gated_ploc/options.h"
#include "morton/utils.h"
#include "ploc/NearestSearch.h"

namespace {

// A unit box at `x` along the x axis, at the given code.
frontier::Entry make_entry(const double x, const morton::Code morton) {
    const glm::dvec3 min{x, 0.0, 0.0};
    return frontier::Entry{
        .morton = morton,
        .header = make_node_header({0.0, 0.0}, bvh::Bounds{min, min + 1.0}),
        .ref = bvh::Reference{},
    };
}

}

TEST_CASE("merge_chunk passes on only the entries and pairs it owns", "[gated_ploc][ChunkMerger]") {
    // A search radius of 1 needs a halo of 2: the chunk owns 20 and 30, whose partners 10 and 40 lie in its halo.
    const std::vector<frontier::Entry> entries{
        make_entry(-100.0, 5),
        make_entry(0.0, 10),
        make_entry(1.0, 20),
        make_entry(100.0, 30),
        make_entry(101.0, 40),
        make_entry(200.0, 50),
    };
    const PaddedSpan<const frontier::Entry> chunk(entries, 2, 2);

    const TempDirectory directory("chunk_merger");
    bvh::Storage output = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "output"));
    bvh::BatchWriter sink(output);
    ploc::NearestSearch search(1, build::DEFAULT_ERROR_WEIGHT);

    std::vector<frontier::Entry> next_run;
    REQUIRE_VALUE(gated_ploc::detail::merge_chunk(search, sink, chunk, next_run));
    REQUIRE_VALUE(sink.flush());
    REQUIRE_VALUE(output.save_index());

    CHECK(to_vector(next_run | std::views::transform(&frontier::Entry::morton)) == std::vector<morton::Code>{30});
}

TEST_CASE("ChunkMerger passes on what merging the stream as one chunk does", "[gated_ploc][ChunkMerger]") {
    // Two touching pairs and a lone box, far apart from each other.
    const std::vector<frontier::Entry> entries{
        make_entry(0.0, 10),
        make_entry(1.0, 20),
        make_entry(100.0, 30),
        make_entry(101.0, 40),
        make_entry(200.0, 50),
    };

    const TempDirectory directory("chunk_merger");
    bvh::Storage reference_output = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "reference_output"));
    bvh::BatchWriter reference_sink(reference_output);
    ploc::NearestSearch search(1, build::DEFAULT_ERROR_WEIGHT);
    std::vector<frontier::Entry> whole;
    REQUIRE_VALUE(gated_ploc::detail::merge_chunk(search, reference_sink, PaddedSpan<const frontier::Entry>(entries, 0, 0), whole));
    REQUIRE_VALUE(reference_sink.flush());
    REQUIRE_VALUE(reference_output.save_index());

    bvh::Storage output = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "output"));
    bvh::BatchWriter sink(output);

    std::vector<morton::Code> codes;
    bool finished = false;
    gated_ploc::detail::ChunkMerger merger(sink, CollectingSink{codes, finished}, gated_ploc::Options{.search_radius = 1, .min_chunk_size = 1});

    SECTION("pushed at once") {
        REQUIRE_VALUE(merger.push(entries));
    }
    // A search radius of 1 needs a halo of 2, so one entry at a time cuts the stream after 10, 20 and 30.
    SECTION("pushed one at a time") {
        for (const frontier::Entry &entry : entries) {
            REQUIRE_VALUE(merger.push(std::span(&entry, 1)));
        }
    }
    REQUIRE_VALUE(std::move(merger).finish());
    REQUIRE_VALUE(sink.flush());
    REQUIRE_VALUE(output.save_index());

    CHECK(codes == to_vector(whole | std::views::transform(&frontier::Entry::morton)));
    CHECK(finished);
}
