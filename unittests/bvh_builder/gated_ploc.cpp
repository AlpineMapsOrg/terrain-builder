
#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "node_header_helpers.h"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "enumerate.h"
#include "error/Result.h"
#include "range_utils.h"

#include "gated_ploc/build.h"
#include "bvh/BatchWriter.h"
#include "bvh/NodeCache.h"
#include "bvh/Storage.h"
#include "bvh/traverse_tree.h"
#include "frontier/Storage.h"
#include "frontier/sort.h"
#include "morton/utils.h"

namespace {

const bvh::Bounds SCENE_BOUNDS{glm::dvec3{0.0}, glm::dvec3{256.0}};

// A frontier over leaves, with the references of the leaves in order.
struct LeafFrontier {
    frontier::Storage frontier;
    std::vector<bvh::Reference> refs;
};

// Stores a leaf for each of `boxes` in `output` and returns the frontier over them as a single sorted run under `directory`.
LeafFrontier write_frontier(const std::filesystem::path &directory, bvh::Storage &output, const std::span<const bvh::Bounds> boxes) {
    bvh::BatchWriter writer(output);
    frontier::Batch batch;
    std::vector<bvh::Reference> refs;
    for (const bvh::Bounds &box : boxes) {
        const bvh::NodeHeader header = make_node_header({0.0, 0.0}, box);
        const bvh::Reference ref = REQUIRE_VALUE(writer.write_node(make_leaf_input(header)));
        batch.entries.push_back(frontier::Entry{.morton = morton::encode(SCENE_BOUNDS, header), .header = header, .ref = ref});
        refs.push_back(ref);
    }
    REQUIRE(writer.flush().has_value());
    REQUIRE(output.save_index().has_value());
    frontier::sort_entries(batch.entries);

    frontier::Storage run = REQUIRE_VALUE(frontier::Storage::create(directory));
    REQUIRE(run.write(batch).has_value());
    REQUIRE(run.save_index().has_value());
    return LeafFrontier{.frontier = std::move(run), .refs = std::move(refs)};
}

// Options under which admission never holds a leaf back, so that only the chunking differs between them.
gated_ploc::Options make_options(const size_t min_chunk_size = gated_ploc::Options{}.min_chunk_size) {
    return gated_ploc::Options{
        .search_radius = 2,
        .admission = gated_ploc::AdmissionOptions{.region_size = 8, .tolerance = 1e9},
        .min_chunk_size = min_chunk_size,
    };
}

// The tree at `root` in pre-order, with each inner node as nothing and each leaf as its position in `leaves`.
std::vector<std::optional<size_t>> read_shape(const bvh::Storage &output, const bvh::Reference &root, const std::span<const bvh::Reference> leaves) {
    bvh::NodeCache cache(output, 1);
    std::vector<std::optional<size_t>> shape;
    const auto result = bvh::traverse_tree(cache, root, [&](const bvh::Reference &reference, const bvh::NodeView &view) {
        if (view.is_inner()) {
            shape.push_back(std::nullopt);
            return;
        }
        for (const auto &[number, leaf] : enumerate(leaves)) {
            if (leaf == reference) {
                shape.push_back(number);
                return;
            }
        }
        FAIL("the tree holds a leaf that was not written");
    });
    REQUIRE(result.value_or(false));
    return shape;
}

// `count` unit boxes in a row.
std::vector<bvh::Bounds> make_row(const uint32_t count) {
    return transform_vector(range(count), [](const uint32_t i) {
        const glm::dvec3 min{static_cast<double>(i), 0.0, 0.0};
        return bvh::Bounds{min, min + 1.0};
    });
}

}

TEST_CASE("a single leaf is the whole tree", "[build][gated_ploc]") {
    const std::vector<bvh::Bounds> leaves{{glm::dvec3{0.0}, glm::dvec3{1.0}}};

    const TempDirectory directory("gated_ploc");
    bvh::Storage output = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "output"));
    auto [frontier, refs] = write_frontier(directory.path() / "frontier", output, leaves);

    const auto root = gated_ploc::build(std::move(frontier), output, directory.path() / "scratch", make_options());
    REQUIRE(root.has_value());

    CHECK(read_shape(output, *root, refs) == std::vector<std::optional<size_t>>{0});
}

TEST_CASE("neighbours are paired before distant nodes are joined", "[build][gated_ploc]") {
    // Two pairs of touching boxes, the pairs far apart from each other.
    const std::vector<bvh::Bounds> leaves{
        {glm::dvec3{0.0}, glm::dvec3{1.0}},
        {glm::dvec3{1.0}, glm::dvec3{2.0}},
        {glm::dvec3{100.0}, glm::dvec3{101.0}},
        {glm::dvec3{101.0}, glm::dvec3{102.0}},
    };

    const TempDirectory directory("gated_ploc");
    bvh::Storage output = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "output"));
    auto [frontier, refs] = write_frontier(directory.path() / "frontier", output, leaves);

    const auto root = gated_ploc::build(std::move(frontier), output, directory.path() / "scratch", make_options());
    REQUIRE(root.has_value());

    const std::vector<std::optional<size_t>> expected{std::nullopt, std::nullopt, 0, 1, std::nullopt, 2, 3};
    CHECK(read_shape(output, *root, refs) == expected);
}

TEST_CASE("the tree holds every leaf exactly once", "[build][gated_ploc]") {
    const uint32_t count = GENERATE(2u, 3u, 8u, 17u);

    const TempDirectory directory("gated_ploc");
    bvh::Storage output = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "output"));
    auto [frontier, refs] = write_frontier(directory.path() / "frontier", output, make_row(count));

    const auto root = gated_ploc::build(std::move(frontier), output, directory.path() / "scratch", make_options());
    REQUIRE(root.has_value());

    const std::vector<std::optional<size_t>> shape = read_shape(output, *root, refs);
    std::vector<size_t> leaf_numbers;
    for (const std::optional<size_t> &node : shape) {
        if (node.has_value()) {
            leaf_numbers.push_back(*node);
        }
    }
    std::ranges::sort(leaf_numbers);

    // Every inner node joins two subtrees, and joining count leaves into one takes count - 1 of them.
    CHECK(shape.size() - leaf_numbers.size() == count - 1);
    CHECK(leaf_numbers == to_vector(range(size_t{count})));
}

TEST_CASE("splitting the frontier into chunks does not change the tree", "[build][gated_ploc]") {
    const std::vector<bvh::Bounds> leaves = make_row(24);

    const TempDirectory single_directory("gated_ploc");
    bvh::Storage single_output = REQUIRE_VALUE(bvh::Storage::create(single_directory.path() / "output"));
    auto [single_frontier, single_refs] = write_frontier(single_directory.path() / "frontier", single_output, leaves);
    const auto single_root = gated_ploc::build(std::move(single_frontier), single_output, single_directory.path() / "scratch", make_options());
    REQUIRE(single_root.has_value());

    const TempDirectory split_directory("gated_ploc");
    bvh::Storage split_output = REQUIRE_VALUE(bvh::Storage::create(split_directory.path() / "output"));
    auto [split_frontier, split_refs] = write_frontier(split_directory.path() / "frontier", split_output, leaves);
    const auto split_root = gated_ploc::build(std::move(split_frontier), split_output, split_directory.path() / "scratch", make_options(8));
    REQUIRE(split_root.has_value());

    CHECK(read_shape(split_output, *split_root, split_refs) == read_shape(single_output, *single_root, single_refs));
}

TEST_CASE("the build writes its root into the last batch", "[build][gated_ploc]") {
    const TempDirectory directory("gated_ploc");
    bvh::Storage output = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "output"));
    auto [frontier, refs] = write_frontier(directory.path() / "frontier", output, make_row(24));

    const bvh::Reference root = REQUIRE_VALUE(gated_ploc::build(std::move(frontier), output, directory.path() / "scratch", make_options()));

    CHECK(root.batch == output.size() - 1);
}

TEST_CASE("a frontier that cannot be read fails the build", "[build][gated_ploc]") {
    const TempDirectory directory("gated_ploc");
    bvh::Storage output = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "output"));
    auto [frontier, refs] = write_frontier(directory.path() / "frontier", output, make_row(4));

    // The storage still counts its batch, but it can no longer be read.
    std::filesystem::remove_all(directory.path() / "frontier");

    const auto root = gated_ploc::build(std::move(frontier), output, directory.path() / "scratch", make_options());
    CHECK_FALSE(root.has_value());
}
