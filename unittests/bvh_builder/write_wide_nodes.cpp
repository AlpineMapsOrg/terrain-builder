#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "node_header_helpers.h"

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include <glm/glm.hpp>

#include "range_utils.h"

#include "bvh/BatchWriter.h"
#include "bvh/NodeCache.h"
#include "bvh/Storage.h"
#include "bvh/NodeHeader.h"
#include "bvh/Reference.h"
#include "widen/options.h"
#include "widen/tabulate_splits.h"
#include "widen/write_wide_nodes.h"

namespace {

// Stores the split rows of R, A and B in the order the tabulation leaves them: B, A, then R.
void write_splits(
    widen::SplitStorage &splits, const std::span<const widen::Split> root_row,
    const std::span<const widen::Split> a_row, const std::span<const widen::Split> b_row) {
    widen::SplitBatch batch;
    batch.splits.push_row(b_row);
    batch.splits.push_row(a_row);
    batch.splits.push_row(root_row);
    REQUIRE_VALUE(splits.write(batch));
    REQUIRE_VALUE(splits.save_index());
}

}

TEST_CASE("a kept root collects every leaf its dissolved children held", "[widen][write_wide_nodes]") {
    // The leaves differ only by their headers, which widening carries over.
    const bvh::Bounds box{glm::dvec3{0.0}, glm::dvec3{1.0}};
    const std::array leaf_headers{
        make_node_header({0.0, 0.0}, box),
        make_node_header({1.0, 1.0}, box),
        make_node_header({2.0, 2.0}, box),
        make_node_header({3.0, 3.0}, box),
    };

    const TempDirectory directory("write_wide_nodes");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "input"));
    bvh::BatchWriter writer(nodes);
    const bvh::Reference leaf_0 = REQUIRE_VALUE(writer.write_node(make_leaf_input(leaf_headers[0])));
    const bvh::Reference leaf_1 = REQUIRE_VALUE(writer.write_node(make_leaf_input(leaf_headers[1])));
    const bvh::Reference leaf_2 = REQUIRE_VALUE(writer.write_node(make_leaf_input(leaf_headers[2])));
    const bvh::Reference leaf_3 = REQUIRE_VALUE(writer.write_node(make_leaf_input(leaf_headers[3])));
    const bvh::Reference a = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{leaf_0, leaf_1}}));
    const bvh::Reference b = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{leaf_2, leaf_3}}));
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{a, b}}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    // The root keeps itself and offers A and B two slots each, at which both dissolve.
    widen::SplitStorage splits = REQUIRE_VALUE(widen::SplitStorage::create(directory.path() / "splits"));
    const std::array root_row{widen::Split::keep(2), widen::Split::dissolve(1), widen::Split::dissolve(1)};
    const std::array child_row{widen::Split::keep(1)};
    write_splits(splits, root_row, child_row, child_row);

    bvh::Storage output = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "output"));
    bvh::NodeCache cache(nodes, 1);
    const bvh::Reference wide_root = REQUIRE_VALUE(widen::write_wide_nodes(splits, cache, root, output, widen::Options{.arity = 4}));
    REQUIRE_VALUE(output.save_index());

    const bvh::Batch root_batch = REQUIRE_VALUE(output.read(wide_root.batch));
    std::vector<bvh::NodeHeader> child_headers;
    for (const bvh::Reference &child : root_batch.resolve(root_batch.node(wide_root.node)).children()) {
        const bvh::Batch child_batch = REQUIRE_VALUE(output.read(child.batch));
        child_headers.push_back(child_batch.node(child.node).header());
    }
    CHECK(child_headers == to_vector(leaf_headers));
}

TEST_CASE("a kept node keeps the header it had", "[widen][write_wide_nodes]") {
    const bvh::NodeHeader root_header = make_node_header({1.0, 2.0}, {glm::dvec3{0.0}, glm::dvec3{4.0}});

    const TempDirectory directory("write_wide_nodes");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "input"));
    bvh::BatchWriter writer(nodes);
    const bvh::Reference leaf_0 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference leaf_1 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference leaf_2 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference leaf_3 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference a = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{leaf_0, leaf_1}}));
    const bvh::Reference b = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{leaf_2, leaf_3}}));
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{root_header, std::array{a, b}}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    widen::SplitStorage splits = REQUIRE_VALUE(widen::SplitStorage::create(directory.path() / "splits"));
    const std::array root_row{widen::Split::keep(2), widen::Split::dissolve(1), widen::Split::dissolve(1)};
    const std::array child_row{widen::Split::keep(1)};
    write_splits(splits, root_row, child_row, child_row);

    bvh::Storage output = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "output"));
    bvh::NodeCache cache(nodes, 1);
    const bvh::Reference wide_root = REQUIRE_VALUE(widen::write_wide_nodes(splits, cache, root, output, widen::Options{.arity = 4}));
    REQUIRE_VALUE(output.save_index());

    const bvh::Batch root_batch = REQUIRE_VALUE(output.read(wide_root.batch));
    CHECK(root_batch.node(wide_root.node).header() == root_header);
}

TEST_CASE("children that keep their slot stay nodes of their own", "[widen][write_wide_nodes]") {
    const TempDirectory directory("write_wide_nodes");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "input"));
    bvh::BatchWriter writer(nodes);
    const bvh::Reference leaf_0 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference leaf_1 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference leaf_2 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference leaf_3 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference a = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{leaf_0, leaf_1}}));
    const bvh::Reference b = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{leaf_2, leaf_3}}));
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{a, b}}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    // At arity two nothing can be widened, so every inner node keeps its slot and its row is one entry long.
    widen::SplitStorage splits = REQUIRE_VALUE(widen::SplitStorage::create(directory.path() / "splits"));
    const std::array row{widen::Split::keep(1)};
    write_splits(splits, row, row, row);

    bvh::Storage output = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "output"));
    bvh::NodeCache cache(nodes, 1);
    const bvh::Reference wide_root = REQUIRE_VALUE(widen::write_wide_nodes(splits, cache, root, output, widen::Options{.arity = 2}));
    REQUIRE_VALUE(output.save_index());

    const bvh::Batch root_batch = REQUIRE_VALUE(output.read(wide_root.batch));
    const std::span<const bvh::Reference> children = root_batch.resolve(root_batch.node(wide_root.node)).children();
    REQUIRE(children.size() == 2);
    for (const bvh::Reference &child : children) {
        const bvh::Batch child_batch = REQUIRE_VALUE(output.read(child.batch));
        CHECK(child_batch.resolve(child_batch.node(child.node)).children().size() == 2);
    }
}