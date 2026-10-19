#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "node_header_helpers.h"
 
#include <vector>

#include "bvh/utils.h"
#include "frontier/build.h"
#include "frontier/Storage.h"
#include "frontier/traverse.h"

namespace {

bvh::Bounds make_cube(const glm::dvec3 &centre, const double size) {
    return bvh::Bounds{centre - size * 0.5, centre + size * 0.5};
}

// One leaf per bounds, all in a single batch, and returns the headers as written.
std::vector<bvh::NodeHeader> write_nodes(bvh::Storage &nodes, const std::vector<bvh::Bounds> &bounds) {
    bvh::Batch batch = nodes.create_batch();
    std::vector<bvh::NodeHeader> written;
    for (const auto &[index, node_bounds] : enumerate<uint32_t>(bounds)) {
        written.push_back(make_node_header({1.0, 2.0}, node_bounds));
        batch.add_node(make_leaf_input(written.back()));
    }
    REQUIRE(nodes.write(batch).has_value());
    REQUIRE(nodes.save_index().has_value());
    return written;
}

std::vector<frontier::Entry> read_run(const frontier::Storage &run) {
    std::vector<frontier::Entry> entries;
    const auto result = frontier::traverse_entries(run, [&](const frontier::Entry &entry) { entries.push_back(entry); });
    REQUIRE(result.has_value());
    return entries;
}

}

TEST_CASE("every stored node enters the frontier carrying its header and reference", "[frontier][build]") {
    const TempDirectory directory("build_frontier");
    std::filesystem::create_directories(directory.path() / "nodes");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));
    const std::vector<bvh::NodeHeader> written = write_nodes(nodes, {make_cube(glm::dvec3{0.25}, 0.1), make_cube(glm::dvec3{0.75}, 0.1)});

    std::filesystem::create_directories(directory.path() / "frontier");
    frontier::Storage run = REQUIRE_VALUE(frontier::Storage::create(directory.path() / "frontier"));
    REQUIRE(frontier::build_from_leaves(nodes, run).has_value());
    REQUIRE(run.save_index().has_value());

    const std::vector<frontier::Entry> entries = read_run(run);
    REQUIRE(entries.size() == 2);
    for (const auto &[index, entry] : enumerate<uint32_t>(entries)) {
        CHECK(entry.ref.batch == 0);
        CHECK(entry.ref.node == index);
        CHECK(entry.header == written[index]);
    }
    CHECK(entries[0].morton < entries[1].morton);
}

TEST_CASE("the entry's morton code comes from the bounds centre", "[frontier][build]") {
    const TempDirectory directory("build_frontier");
    std::filesystem::create_directories(directory.path() / "nodes");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));

    // Same centre, different extent, so only a code taken at the centre makes these agree.
    write_nodes(nodes, {
        make_cube(glm::dvec3{0.25}, 0.1),
        make_cube(glm::dvec3{0.25}, 0.4),
        make_cube(glm::dvec3{0.75}, 0.1),
    });

    std::filesystem::create_directories(directory.path() / "frontier");
    frontier::Storage run = REQUIRE_VALUE(frontier::Storage::create(directory.path() / "frontier"));
    REQUIRE(frontier::build_from_leaves(nodes, run).has_value());
    REQUIRE(run.save_index().has_value());

    const std::vector<frontier::Entry> entries = read_run(run);
    REQUIRE(entries.size() == 3);
    CHECK(entries[0].morton == entries[1].morton);
    CHECK(entries[0].morton != entries[2].morton);
}

TEST_CASE("the byte budget splits the frontier into batches", "[frontier][build]") {
    const TempDirectory directory("build_frontier");
    std::filesystem::create_directories(directory.path() / "nodes");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));

    std::vector<bvh::Bounds> bounds;
    for (uint32_t i = 0; i < 5; i++) {
        bounds.push_back(make_cube(glm::dvec3{0.1 * (i + 1)}, 0.05));
    }
    write_nodes(nodes, bounds);

    std::filesystem::create_directories(directory.path() / "frontier");
    frontier::Storage run = REQUIRE_VALUE(frontier::Storage::create(directory.path() / "frontier"));
    const frontier::BuildOptions options{.max_batch_bytes = 2 * sizeof(frontier::Entry)};
    REQUIRE(frontier::build_from_leaves(nodes, run, options).has_value());
    REQUIRE(run.save_index().has_value());

    CHECK(run.size() == 3);
    CHECK(read_run(run).size() == 5);
}

TEST_CASE("an empty store yields an empty frontier", "[frontier][build]") {
    const TempDirectory directory("build_frontier");
    std::filesystem::create_directories(directory.path() / "nodes");
    const bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));

    std::filesystem::create_directories(directory.path() / "frontier");
    frontier::Storage run = REQUIRE_VALUE(frontier::Storage::create(directory.path() / "frontier"));
    REQUIRE(frontier::build_from_leaves(nodes, run).has_value());
    REQUIRE(run.save_index().has_value());

    CHECK(run.size() == 0);
}