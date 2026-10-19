#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "node_header_helpers.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <vector>

#include <glm/glm.hpp>

#include "range_utils.h"

#include "pseudo_lbvh/build.h"
#include "bvh/BatchWriter.h"
#include "bvh/Storage.h"
#include "frontier/Storage.h"
#include "storage/batch_limits.h"

namespace {

const morton::Code TOP_BIT = morton::Code{1} << (morton::CODE_BITS - 1);

// An entry holding nothing but `code`, for the tests that only look at codes.
frontier::Entry make_entry(const morton::Code &code) {
    return frontier::Entry{
        .morton = code,
        .header = {},
        .ref = {},
    };
}

// `count` codes in ascending order, spread out so that runs split at several bit depths.
std::vector<morton::Code> make_ascending_codes(const uint32_t count) {
    return transform_vector(range(count), [](const uint32_t i) { return morton::Code{i} << 7; });
}

// The children of the inner node `nodes` holds at `reference`.
std::vector<bvh::Reference> read_children(const bvh::Storage &nodes, const bvh::Reference &reference) {
    const auto batch = nodes.read(reference.batch);
    REQUIRE(batch.has_value());
    return to_vector(batch->resolve(batch->node(reference.node)).children());
}

}

TEST_CASE("a prefix keeps the leading bits of a code and clears the rest", "[pseudo_lbvh][build]") {
    const morton::Code all_ones = std::numeric_limits<morton::Code>::max();

    CHECK(pseudo_lbvh::detail::extract_prefix(all_ones, 0) == morton::Code{0});
    CHECK(pseudo_lbvh::detail::extract_prefix(all_ones, morton::CODE_BITS) == all_ones);
    CHECK(pseudo_lbvh::detail::extract_prefix(all_ones, 1) == TOP_BIT);
    CHECK(pseudo_lbvh::detail::extract_prefix(morton::Code{1}, morton::CODE_BITS - 1) == morton::Code{0});

    for (const uint32_t bits : range<uint32_t>(1, morton::CODE_BITS)) {
        const morton::Code kept = all_ones << (morton::CODE_BITS - bits);
        REQUIRE(pseudo_lbvh::detail::extract_prefix(all_ones, bits) == kept);
    }
}

TEST_CASE("a run splits at the highest bit its codes disagree on", "[pseudo_lbvh][build]") {
    const std::vector<frontier::Entry> low = transform_vector(std::array{morton::Code{0}, morton::Code{1}}, make_entry);
    CHECK(pseudo_lbvh::detail::find_split(low) == 1);

    const std::vector<frontier::Entry> high = transform_vector(std::array{morton::Code{0}, morton::Code{1}, TOP_BIT, TOP_BIT | morton::Code{1}}, make_entry);
    CHECK(pseudo_lbvh::detail::find_split(high) == 2);
}

TEST_CASE("a run of equal codes splits in half", "[pseudo_lbvh][build]") {
    CHECK(pseudo_lbvh::detail::find_split(std::vector<frontier::Entry>(4, make_entry(morton::Code{7}))) == 2);
    CHECK(pseudo_lbvh::detail::find_split(std::vector<frontier::Entry>(5, make_entry(morton::Code{7}))) == 2);
}

TEST_CASE("a split never leaves either side empty", "[pseudo_lbvh][build]") {
    const std::vector<std::vector<morton::Code>> runs{
        std::vector<morton::Code>(2, morton::Code{0}),
        std::vector<morton::Code>(9, morton::Code{3}),
        make_ascending_codes(2),
        make_ascending_codes(3),
        make_ascending_codes(16),
        make_ascending_codes(17),
        {morton::Code{0}, morton::Code{0}, morton::Code{1}, morton::Code{1}, morton::Code{1}},
        {morton::Code{0}, TOP_BIT, TOP_BIT, TOP_BIT | morton::Code{1}},
    };

    for (const std::vector<morton::Code> &codes : runs) {
        const std::vector<frontier::Entry> entries = transform_vector(codes, make_entry);
        const size_t split = pseudo_lbvh::detail::find_split(entries);

        REQUIRE(split > 0);
        REQUIRE(split < entries.size());
    }
}

TEST_CASE("a single entry is its own root and writes nothing", "[pseudo_lbvh][build]") {
    const frontier::Entry entry{.morton = morton::Code{0b1}, .header = {}, .ref = {.batch = 100, .node = 0}};

    const TempDirectory directory("pseudo_lbvh");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const auto root = pseudo_lbvh::detail::build_subtree(std::array{entry}, writer);
    REQUIRE(root.has_value());
    REQUIRE(writer.flush().has_value());

    CHECK(*root == entry);
    CHECK(nodes.size() == 0);
}

TEST_CASE("two entries are joined under a parent carrying both their headers", "[pseudo_lbvh][build]") {
    // The references name leaves stored elsewhere; the builder never reads them.
    const frontier::Entry left{
        .morton = morton::Code{0b01},
        .header = make_node_header({1.0, 3.0}, {glm::dvec3{0.0}, glm::dvec3{1.0}}),
        .ref = {.batch = 100, .node = 0},
    };
    const frontier::Entry right{
        .morton = morton::Code{0b10},
        .header = make_node_header({2.0, 4.0}, {glm::dvec3{5.0}, glm::dvec3{7.0}}),
        .ref = {.batch = 100, .node = 1},
    };

    const TempDirectory directory("pseudo_lbvh");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const auto root = pseudo_lbvh::detail::build_subtree(std::array{left, right}, writer);
    REQUIRE(root.has_value());
    REQUIRE(writer.flush().has_value());
    REQUIRE_VALUE(nodes.save_index());

    CHECK(root->morton == left.morton);
    CHECK(root->header == make_node_header({1.0, 4.0}, {glm::dvec3{0.0}, glm::dvec3{7.0}}));
    CHECK(read_children(nodes, root->ref) == std::vector{left.ref, right.ref});
}

TEST_CASE("entries whose codes differ in the top bit end up in separate subtrees", "[pseudo_lbvh][build]") {
    const std::array entries{
        frontier::Entry{.morton = morton::Code{0}, .header = {}, .ref = {.batch = 100, .node = 0}},
        frontier::Entry{.morton = morton::Code{1}, .header = {}, .ref = {.batch = 100, .node = 1}},
        frontier::Entry{.morton = TOP_BIT, .header = {}, .ref = {.batch = 100, .node = 2}},
        frontier::Entry{.morton = TOP_BIT | morton::Code{1}, .header = {}, .ref = {.batch = 100, .node = 3}},
    };

    const TempDirectory directory("pseudo_lbvh");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const auto root = pseudo_lbvh::detail::build_subtree(entries, writer);
    REQUIRE(root.has_value());
    REQUIRE(writer.flush().has_value());
    REQUIRE_VALUE(nodes.save_index());

    const std::vector<bvh::Reference> children = read_children(nodes, root->ref);
    REQUIRE(children.size() == 2);
    CHECK(read_children(nodes, children[0]) == std::vector{entries[0].ref, entries[1].ref});
    CHECK(read_children(nodes, children[1]) == std::vector{entries[2].ref, entries[3].ref});
}

TEST_CASE("entries with equal codes are still all joined, halving the run", "[pseudo_lbvh][build]") {
    const std::array entries{
        frontier::Entry{.morton = morton::Code{7}, .header = {}, .ref = {.batch = 100, .node = 0}},
        frontier::Entry{.morton = morton::Code{7}, .header = {}, .ref = {.batch = 100, .node = 1}},
        frontier::Entry{.morton = morton::Code{7}, .header = {}, .ref = {.batch = 100, .node = 2}},
    };

    const TempDirectory directory("pseudo_lbvh");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const auto root = pseudo_lbvh::detail::build_subtree(entries, writer);
    REQUIRE(root.has_value());
    REQUIRE(writer.flush().has_value());
    REQUIRE_VALUE(nodes.save_index());

    const std::vector<bvh::Reference> children = read_children(nodes, root->ref);
    REQUIRE(children.size() == 2);
    CHECK(children[0] == entries[0].ref);
    CHECK(read_children(nodes, children[1]) == std::vector{entries[1].ref, entries[2].ref});
}

TEST_CASE("each batch of a run gets a subtree of its own before their roots are joined", "[pseudo_lbvh][build]") {
    const std::array entries{
        frontier::Entry{.morton = morton::Code{0}, .header = {}, .ref = {.batch = 100, .node = 0}},
        frontier::Entry{.morton = TOP_BIT, .header = {}, .ref = {.batch = 100, .node = 1}},
        frontier::Entry{.morton = TOP_BIT | morton::Code{1}, .header = {}, .ref = {.batch = 100, .node = 2}},
    };

    const TempDirectory directory("pseudo_lbvh");
    // A single run would pair the last two; the batch boundary pairs the first two instead.
    frontier::Storage run = REQUIRE_VALUE(frontier::Storage::create(directory.path() / "run"));
    REQUIRE(run.write(frontier::Batch{.entries = {entries[0], entries[1]}}).has_value());
    REQUIRE(run.write(frontier::Batch{.entries = {entries[2]}}).has_value());
    REQUIRE(run.save_index().has_value());

    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));
    bvh::BatchWriter writer(nodes);
    const auto root = pseudo_lbvh::build(run, writer);
    REQUIRE(root.has_value());
    REQUIRE(writer.flush().has_value());
    REQUIRE_VALUE(nodes.save_index());

    const std::vector<bvh::Reference> children = read_children(nodes, root->ref);
    REQUIRE(children.size() == 2);
    CHECK(read_children(nodes, children[0]) == std::vector{entries[0].ref, entries[1].ref});
    CHECK(children[1] == entries[2].ref);
}

TEST_CASE("building into a storage saves its index", "[pseudo_lbvh][build]") {
    const TempDirectory directory("pseudo_lbvh");
    frontier::Storage run = REQUIRE_VALUE(frontier::Storage::create(directory.path() / "run"));
    REQUIRE(run.write(frontier::Batch{.entries = {
        frontier::Entry{.morton = morton::Code{0}, .header = {}, .ref = {.batch = 100, .node = 0}},
        frontier::Entry{.morton = morton::Code{1}, .header = {}, .ref = {.batch = 100, .node = 1}},
    }}).has_value());
    REQUIRE(run.save_index().has_value());

    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));
    REQUIRE(pseudo_lbvh::build(run, nodes, pseudo_lbvh::Options{}).has_value());

    const bvh::Storage reopened = REQUIRE_VALUE(bvh::Storage::open(directory.path() / "nodes"));
    CHECK(reopened.size() == 1);
}

TEST_CASE("a batch limit below one node writes every inner node into a batch of its own", "[pseudo_lbvh][build]") {
    const TempDirectory directory("pseudo_lbvh");
    frontier::Storage run = REQUIRE_VALUE(frontier::Storage::create(directory.path() / "run"));
    REQUIRE(run.write(frontier::Batch{.entries = {
        frontier::Entry{.morton = morton::Code{0}, .header = {}, .ref = {.batch = 100, .node = 0}},
        frontier::Entry{.morton = morton::Code{1}, .header = {}, .ref = {.batch = 100, .node = 1}},
        frontier::Entry{.morton = morton::Code{2}, .header = {}, .ref = {.batch = 100, .node = 2}},
        frontier::Entry{.morton = morton::Code{3}, .header = {}, .ref = {.batch = 100, .node = 3}},
    }}).has_value());
    REQUIRE(run.save_index().has_value());

    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));
    REQUIRE(pseudo_lbvh::build(run, nodes, pseudo_lbvh::Options{.max_batch_bytes = 1}).has_value());

    // Four entries take three inner nodes.
    CHECK(nodes.size() == 3);
}

TEST_CASE("the build writes its root into the last batch", "[pseudo_lbvh][build]") {
    const size_t max_batch_bytes = GENERATE(DEFAULT_MAX_BATCH_BYTES, size_t{1});

    const TempDirectory directory("pseudo_lbvh");
    frontier::Storage run = REQUIRE_VALUE(frontier::Storage::create(directory.path() / "run"));
    REQUIRE(run.write(frontier::Batch{.entries = {
        frontier::Entry{.morton = morton::Code{0}, .header = {}, .ref = {.batch = 100, .node = 0}},
        frontier::Entry{.morton = morton::Code{1}, .header = {}, .ref = {.batch = 100, .node = 1}},
        frontier::Entry{.morton = morton::Code{2}, .header = {}, .ref = {.batch = 100, .node = 2}},
        frontier::Entry{.morton = morton::Code{3}, .header = {}, .ref = {.batch = 100, .node = 3}},
    }}).has_value());
    REQUIRE(run.save_index().has_value());

    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));
    const bvh::Reference root = REQUIRE_VALUE(pseudo_lbvh::build(run, nodes, pseudo_lbvh::Options{.max_batch_bytes = max_batch_bytes}));

    CHECK(root.batch == nodes.size() - 1);
}
