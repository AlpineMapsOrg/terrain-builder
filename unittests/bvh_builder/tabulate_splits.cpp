#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "node_header_helpers.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <vector>

#include <glm/glm.hpp>

#include "io/serde.h"
#include "range_utils.h"

#include "bvh/BatchWriter.h"
#include "bvh/NodeCache.h"
#include "bvh/Storage.h"
#include "bvh/Batch.h"
#include "bvh/NodeHeader.h"
#include "bvh/Reference.h"
#include "widen/options.h"
#include "widen/tabulate_splits.h"

namespace {

// A header without error over the box from `min` to `max`.
bvh::NodeHeader make_box_header(const glm::dvec3 &min, const glm::dvec3 &max) {
    return make_node_header({0.0, 0.0}, {min, max});
}

// Tabulates the tree at `root` at `arity` into a storage in `directory` and returns its rows in the order they were written.
std::vector<std::vector<widen::Split>> tabulate_rows(const bvh::Storage &nodes, const bvh::Reference &root, const uint32_t arity, const std::filesystem::path &directory) {
    widen::SplitStorage splits = REQUIRE_VALUE(widen::SplitStorage::create(directory));
    bvh::NodeCache cache(nodes, 1);
    REQUIRE_VALUE(widen::tabulate_splits(cache, root, splits, widen::Options{.arity = arity}));
    REQUIRE_VALUE(splits.save_index());

    std::vector<std::vector<widen::Split>> rows;
    for (const uint32_t index : range(splits.size())) {
        const widen::SplitBatch batch = REQUIRE_VALUE(splits.read(index));
        for (const std::span<const widen::Split> row : batch.splits) {
            rows.emplace_back(row.begin(), row.end());
        }
    }
    return rows;
}

}

TEST_CASE("tabulate_splits makes a node offered one slot keep itself", "[widen][tabulate_splits]") {
    /*
         R
        / \
       0   1
    */
    const TempDirectory directory("tabulate_splits");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));
    bvh::BatchWriter writer(nodes);
    const bvh::Reference leaf_0 = REQUIRE_VALUE(writer.write_node(make_leaf_input(make_box_header({0, 0, 0}, {1, 1, 1}))));
    const bvh::Reference leaf_1 = REQUIRE_VALUE(writer.write_node(make_leaf_input(make_box_header({1, 0, 0}, {2, 1, 1}))));
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{make_box_header({0, 0, 0}, {2, 1, 1}), std::array{leaf_0, leaf_1}}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    // Two leaves can use only one slot, which the root has to fill itself.
    const std::vector<std::vector<widen::Split>> rows = tabulate_rows(nodes, root, 2, directory.path() / "splits");
    CHECK(rows == std::vector<std::vector<widen::Split>>{{widen::Split::keep(1)}});
}

TEST_CASE("tabulate_splits writes one row per inner node, right subtree first, as long as the slots its subtree can use", "[widen][tabulate_splits]") {
    /*
            R
          /   \
         A     C
        / \   / \
       0   1 2   D
                / \
               3   4
    */
    const TempDirectory directory("tabulate_splits");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));
    bvh::BatchWriter writer(nodes);
    const bvh::Reference leaf_0 = REQUIRE_VALUE(writer.write_node(make_leaf_input(make_box_header({0, 0, 0}, {1, 1, 1}))));
    const bvh::Reference leaf_1 = REQUIRE_VALUE(writer.write_node(make_leaf_input(make_box_header({1, 0, 0}, {2, 1, 1}))));
    const bvh::Reference leaf_2 = REQUIRE_VALUE(writer.write_node(make_leaf_input(make_box_header({2, 0, 0}, {3, 1, 1}))));
    const bvh::Reference leaf_3 = REQUIRE_VALUE(writer.write_node(make_leaf_input(make_box_header({3, 0, 0}, {4, 1, 1}))));
    const bvh::Reference leaf_4 = REQUIRE_VALUE(writer.write_node(make_leaf_input(make_box_header({4, 0, 0}, {5, 1, 1}))));
    const bvh::Reference a = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{make_box_header({0, 0, 0}, {2, 1, 1}), std::array{leaf_0, leaf_1}}));
    const bvh::Reference d = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{make_box_header({3, 0, 0}, {5, 1, 1}), std::array{leaf_3, leaf_4}}));
    const bvh::Reference c = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{make_box_header({2, 0, 0}, {5, 1, 1}), std::array{leaf_2, d}}));
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{make_box_header({0, 0, 0}, {5, 1, 1}), std::array{a, c}}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    const auto count_entries = [](const std::vector<std::vector<widen::Split>> &rows) {
        return transform_vector(rows, [](const std::vector<widen::Split> &row) { return row.size(); });
    };

    // A subtree over m leaves can use m - 1 slots, and no node is offered more than arity - 1, giving D, C, A, R.
    CHECK(count_entries(tabulate_rows(nodes, root, 8, directory.path() / "arity_8")) == std::vector<size_t>{1, 2, 1, 4});
    CHECK(count_entries(tabulate_rows(nodes, root, 3, directory.path() / "arity_3")) == std::vector<size_t>{1, 2, 1, 2});
}

TEST_CASE("tabulate_splits gives a kept node's slots to the child that saves the most", "[widen][tabulate_splits]") {
    /*
            R
          /   \
         A     C
        / \   / \
       0   1 2   D
                / \
               3   4
    */
    // A's leaves are 2-unit cubes and the others 1-unit cubes, so D costs 10, C 14 and A 40.
    const TempDirectory directory("tabulate_splits");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));
    bvh::BatchWriter writer(nodes);
    const bvh::Reference leaf_0 = REQUIRE_VALUE(writer.write_node(make_leaf_input(make_box_header({0, 0, 0}, {2, 2, 2}))));
    const bvh::Reference leaf_1 = REQUIRE_VALUE(writer.write_node(make_leaf_input(make_box_header({2, 0, 0}, {4, 2, 2}))));
    const bvh::Reference leaf_2 = REQUIRE_VALUE(writer.write_node(make_leaf_input(make_box_header({4, 0, 0}, {5, 1, 1}))));
    const bvh::Reference leaf_3 = REQUIRE_VALUE(writer.write_node(make_leaf_input(make_box_header({5, 0, 0}, {6, 1, 1}))));
    const bvh::Reference leaf_4 = REQUIRE_VALUE(writer.write_node(make_leaf_input(make_box_header({6, 0, 0}, {7, 1, 1}))));
    const bvh::Reference a = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{make_box_header({0, 0, 0}, {4, 2, 2}), std::array{leaf_0, leaf_1}}));
    const bvh::Reference d = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{make_box_header({5, 0, 0}, {7, 1, 1}), std::array{leaf_3, leaf_4}}));
    const bvh::Reference c = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{make_box_header({4, 0, 0}, {7, 1, 1}), std::array{leaf_2, d}}));
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{make_box_header({0, 0, 0}, {7, 2, 2}), std::array{a, c}}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    // The kept root's four slots can go 1 + 3 (A kept, C dissolved: 40), 2 + 2 (only D kept: 10) or 3 + 1 (only C kept: 14).
    const std::vector<std::vector<widen::Split>> rows = tabulate_rows(nodes, root, 4, directory.path() / "splits");
    REQUIRE(!rows.empty());
    CHECK(rows.back().front() == widen::Split::keep(2));
}

TEST_CASE("tabulate_splits makes a dissolving node keep its cheaper child", "[widen][tabulate_splits]") {
    /*
            R
          /   \
         A     B
        / \   / \
       0   1 2   3
    */
    // A's leaves are 1-unit cubes and B's 2-unit cubes, so A costs 10, B 40 and R 56.
    const TempDirectory directory("tabulate_splits");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));
    bvh::BatchWriter writer(nodes);
    const bvh::Reference leaf_0 = REQUIRE_VALUE(writer.write_node(make_leaf_input(make_box_header({0, 0, 0}, {1, 1, 1}))));
    const bvh::Reference leaf_1 = REQUIRE_VALUE(writer.write_node(make_leaf_input(make_box_header({1, 0, 0}, {2, 1, 1}))));
    const bvh::Reference leaf_2 = REQUIRE_VALUE(writer.write_node(make_leaf_input(make_box_header({2, 0, 0}, {4, 2, 2}))));
    const bvh::Reference leaf_3 = REQUIRE_VALUE(writer.write_node(make_leaf_input(make_box_header({4, 0, 0}, {6, 2, 2}))));
    const bvh::Reference a = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{make_box_header({0, 0, 0}, {2, 1, 1}), std::array{leaf_0, leaf_1}}));
    const bvh::Reference b = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{make_box_header({2, 0, 0}, {6, 2, 2}), std::array{leaf_2, leaf_3}}));
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{make_box_header({0, 0, 0}, {6, 2, 2}), std::array{a, b}}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    // Offered three slots, the root can stay kept (56), or dissolve and keep A (10) or B (40).
    const std::vector<std::vector<widen::Split>> rows = tabulate_rows(nodes, root, 4, directory.path() / "splits");
    REQUIRE(!rows.empty());
    REQUIRE(rows.back().size() == 3);
    CHECK(rows.back()[2] == widen::Split::dissolve(1));
}

TEST_CASE("tabulate_splits fails when a batch cannot be read", "[widen][tabulate_splits]") {
    const TempDirectory directory("tabulate_splits");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));

    // Reserved but never written.
    const bvh::Batch missing = nodes.create_batch();

    bvh::BatchWriter writer(nodes);
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{bvh::Reference{.batch = missing.index(), .node = 0}, bvh::Reference{.batch = missing.index(), .node = 1}}}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    widen::SplitStorage splits = REQUIRE_VALUE(widen::SplitStorage::create(directory.path() / "splits"));
    bvh::NodeCache cache(nodes, 1);
    const io::serde::Error error = REQUIRE_ERROR(widen::tabulate_splits(cache, root, splits, widen::Options{.arity = 4}));
    CHECK(error.holds_and<io::FileError>(&io::FileError::is_not_found));
}
