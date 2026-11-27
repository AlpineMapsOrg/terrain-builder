#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "frontier_helpers.h"
#include "node_header_helpers.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <initializer_list>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <glm/glm.hpp>

#include "enumerate.h"

#include "bvh/BatchWriter.h"
#include "bvh/Storage.h"
#include "bvh/Batch.h"
#include "bvh/NodeHeader.h"
#include "bvh/NodeView.h"
#include "bvh/Reference.h"
#include "frontier/Storage.h"
#include "frontier/layout.h"
#include "hploc/build.h"
#include "morton/code.h"

namespace {

const morton::Code TOP_BIT = morton::Code{1} << (morton::CODE_BITS - 1);

// A tree read back from storage: a leaf's number, or its children in order.
struct Tree {
    Tree() = default;
    Tree(const size_t leaf) : leaf(leaf) {}
    Tree(const std::initializer_list<Tree> children) : children(children) {}

    bool operator==(const Tree &) const = default;

    std::optional<size_t> leaf;
    std::vector<Tree> children;
};

// Formats `tree` as nested braces, with each leaf as its number.
std::string format_as(const Tree &tree) {
    if (tree.leaf.has_value()) {
        return fmt::to_string(*tree.leaf);
    }
    return fmt::format("{{{}}}", fmt::join(tree.children, ", "));
}

struct Leaf {
    morton::Code code;
    bvh::NodeHeader header;
};

// A run over leaves, with the references of the leaves in order.
struct LeafRun {
    frontier::Storage run;
    std::vector<bvh::Reference> refs;
};

// The header of a unit box starting at `x` along the x axis, with `error` as both ends of its error interval.
bvh::NodeHeader make_unit_box_header(const double x, const double error = 0.0) {
    return make_node_header({error, error}, {glm::dvec3{x, 0.0, 0.0}, glm::dvec3{x + 1.0, 1.0, 1.0}});
}

// Stores the leaves of `batches` in `nodes` and returns the run over them.
// The run has one batch per element of `batches`.
LeafRun write_run(const std::filesystem::path &directory, bvh::Storage &nodes, const std::vector<std::vector<Leaf>> &batches) {
    bvh::BatchWriter writer(nodes);
    std::vector<std::vector<frontier::Entry>> entries;
    std::vector<bvh::Reference> refs;
    for (const std::vector<Leaf> &batch : batches) {
        entries.emplace_back();
        for (const Leaf &leaf : batch) {
            const bvh::Reference ref = REQUIRE_VALUE(writer.write_node(make_leaf_input(leaf.header)));
            entries.back().push_back(frontier::Entry{.morton = leaf.code, .header = leaf.header, .ref = ref});
            refs.push_back(ref);
        }
    }
    REQUIRE_VALUE(writer.flush());
    return LeafRun{.run = make_storage(directory, entries), .refs = std::move(refs)};
}

// Reads the tree at `root`, numbering each leaf by its position in `leaves`.
Tree read_tree(const bvh::Storage &nodes, const bvh::Reference &root, const std::span<const bvh::Reference> leaves) {
    const bvh::Batch batch = REQUIRE_VALUE(nodes.read(root.batch));
    const bvh::NodeView node = batch.resolve(batch.node(root.node));
    if (node.is_leaf()) {
        for (const auto &[number, leaf] : enumerate(leaves)) {
            if (leaf == root) {
                return Tree(number);
            }
        }
        FAIL("the tree holds a leaf that was not written");
    }

    Tree tree;
    for (const bvh::Reference &child : node.children()) {
        tree.children.push_back(read_tree(nodes, child, leaves));
    }
    return tree;
}

}

template <>
struct Catch::StringMaker<Tree> {
    static std::string convert(const Tree &tree) {
        return fmt::format("{}", tree);
    }
};

TEST_CASE("build keeps a single entry as the tree", "[hploc][build]") {
    const TempDirectory directory("hploc");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));
    const auto [run, refs] = write_run(directory.path() / "run", nodes, {{Leaf{morton::Code{0}, make_unit_box_header(0.0)}}});
    const size_t leaf_batches = nodes.size();

    const bvh::Reference root = REQUIRE_VALUE(hploc::build(run, nodes, hploc::Options{}));

    CHECK(root == refs[0]);
    // It writes no inner node.
    CHECK(nodes.size() == leaf_batches);
}

TEST_CASE("build joins two entries under one parent", "[hploc][build]") {
    // Each entry holds the smaller value on some axes and the larger on others.
    const Leaf left{morton::Code{0b01}, make_node_header({1.0, 4.0}, {glm::dvec3{0.0, 5.0, 0.0}, glm::dvec3{1.0, 6.0, 3.0}})};
    const Leaf right{morton::Code{0b10}, make_node_header({2.0, 3.0}, {glm::dvec3{2.0, 0.0, 1.0}, glm::dvec3{6.0, 1.0, 2.0}})};

    const TempDirectory directory("hploc");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));
    const auto [run, refs] = write_run(directory.path() / "run", nodes, {{left, right}});

    bvh::BatchWriter sink(nodes);
    const frontier::Entry root = REQUIRE_VALUE(hploc::build(run, sink, hploc::Options{}));
    REQUIRE_VALUE(sink.flush());
    REQUIRE_VALUE(nodes.save_index());

    CHECK(root.header == make_node_header({1.0, 4.0}, {glm::dvec3{0.0, 0.0, 0.0}, glm::dvec3{6.0, 6.0, 3.0}}));
    CHECK(read_tree(nodes, root.ref, refs) == Tree{0, 1});
}

TEST_CASE("build merges a range down to the threshold", "[hploc][build]") {
    // By code, leaves 0 and 1 are closest, then 2, then 3.
    // By distance, leaves 2 and 3 are closest, then 1, then 0.
    const std::vector<Leaf> leaves{
        {morton::Code{0}, make_unit_box_header(0.0)},
        {morton::Code{1}, make_unit_box_header(10.0)},
        {morton::Code{2}, make_unit_box_header(13.0)},
        {TOP_BIT, make_unit_box_header(14.0)},
    };

    const TempDirectory directory("hploc");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));
    const auto [run, refs] = write_run(directory.path() / "run", nodes, {leaves});

    SECTION("threshold 1 follows the codes") {
        const bvh::Reference root = REQUIRE_VALUE(hploc::build(run, nodes, hploc::Options{.merge_threshold = 1}));
        CHECK(read_tree(nodes, root, refs) == Tree{{{0, 1}, 2}, 3});
    }
    SECTION("threshold 2 pairs leaves 1 and 2 first") {
        const bvh::Reference root = REQUIRE_VALUE(hploc::build(run, nodes, hploc::Options{.merge_threshold = 2}));
        CHECK(read_tree(nodes, root, refs) == Tree{0, {{1, 2}, 3}});
    }
    SECTION("threshold 4 follows the distances") {
        const bvh::Reference root = REQUIRE_VALUE(hploc::build(run, nodes, hploc::Options{.merge_threshold = 4}));
        CHECK(read_tree(nodes, root, refs) == Tree{0, {1, {2, 3}}});
    }
}

TEST_CASE("build splits equal codes by position", "[hploc][build]") {
    // At threshold 1 the boxes play no part.
    const bvh::NodeHeader header = make_unit_box_header(0.0);

    const TempDirectory directory("hploc");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));

    SECTION("four equal codes") {
        const std::vector<Leaf> leaves{{morton::Code{7}, header}, {morton::Code{7}, header}, {morton::Code{7}, header}, {morton::Code{7}, header}};
        const auto [run, refs] = write_run(directory.path() / "run", nodes, {leaves});

        const bvh::Reference root = REQUIRE_VALUE(hploc::build(run, nodes, hploc::Options{.merge_threshold = 1}));
        CHECK(read_tree(nodes, root, refs) == Tree{{0, 1}, {2, 3}});
    }
    SECTION("after another code") {
        const std::vector<Leaf> leaves{{morton::Code{0}, header}, {morton::Code{7}, header}, {morton::Code{7}, header}, {morton::Code{7}, header}};
        const auto [run, refs] = write_run(directory.path() / "run", nodes, {leaves});

        const bvh::Reference root = REQUIRE_VALUE(hploc::build(run, nodes, hploc::Options{.merge_threshold = 1}));
        CHECK(read_tree(nodes, root, refs) == Tree{0, {1, {2, 3}}});
    }
}

TEST_CASE("build ignores batch boundaries", "[hploc][build]") {
    // At threshold 1 the boxes play no part.
    const bvh::NodeHeader header = make_unit_box_header(0.0);

    const TempDirectory directory("hploc");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));

    SECTION("distinct codes") {
        const std::vector<Leaf> first{{morton::Code{0}, header}, {morton::Code{1}, header}, {TOP_BIT, header}};
        const std::vector<Leaf> second{{TOP_BIT | morton::Code{1}, header}};
        const auto [run, refs] = write_run(directory.path() / "run", nodes, {first, second});

        const bvh::Reference root = REQUIRE_VALUE(hploc::build(run, nodes, hploc::Options{.merge_threshold = 1}));
        CHECK(read_tree(nodes, root, refs) == Tree{{0, 1}, {2, 3}});
    }
    SECTION("equal codes") {
        const std::vector<Leaf> first{{morton::Code{0}, header}, {morton::Code{7}, header}};
        const std::vector<Leaf> second{{morton::Code{7}, header}, {morton::Code{7}, header}};
        const auto [run, refs] = write_run(directory.path() / "run", nodes, {first, second});

        const bvh::Reference root = REQUIRE_VALUE(hploc::build(run, nodes, hploc::Options{.merge_threshold = 1}));
        CHECK(read_tree(nodes, root, refs) == Tree{0, {1, {2, 3}}});
    }
}

TEST_CASE("build respects the search radius", "[hploc][build]") {
    // Leaves 0 and 2 touch. Leaf 1 lies far off.
    const std::vector<Leaf> leaves{{morton::Code{0}, make_unit_box_header(0.0)}, {morton::Code{1}, make_unit_box_header(100.0)}, {morton::Code{2}, make_unit_box_header(1.0)}};

    const TempDirectory directory("hploc");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));
    const auto [run, refs] = write_run(directory.path() / "run", nodes, {leaves});

    SECTION("at radius 1 leaf 0 cannot reach leaf 2") {
        const bvh::Reference root = REQUIRE_VALUE(hploc::build(run, nodes, hploc::Options{.search_radius = 1}));
        CHECK(read_tree(nodes, root, refs) == Tree{0, {1, 2}});
    }
    SECTION("at radius 2 leaves 0 and 2 pair") {
        const bvh::Reference root = REQUIRE_VALUE(hploc::build(run, nodes, hploc::Options{.search_radius = 2}));
        CHECK(read_tree(nodes, root, refs) == Tree{{0, 2}, 1});
    }
}

TEST_CASE("build respects the error weight", "[hploc][build]") {
    // Leaf 1 lies nearest to leaf 0 but differs from it in error by 10.
    // Joining leaves 0 and 2 costs 18 at any weight.
    // Joining leaves 0 and 1 costs 14 at weight 0.05 and 26 at weight 0.2.
    const std::vector<Leaf> leaves{{morton::Code{0}, make_unit_box_header(0.0)}, {morton::Code{1}, make_unit_box_header(1.0, 10.0)}, {morton::Code{2}, make_unit_box_header(3.0)}};

    const TempDirectory directory("hploc");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));
    const auto [run, refs] = write_run(directory.path() / "run", nodes, {leaves});

    SECTION("at weight 0.05 leaves 0 and 1 pair") {
        const bvh::Reference root = REQUIRE_VALUE(hploc::build(run, nodes, hploc::Options{.error_weight = 0.05}));
        CHECK(read_tree(nodes, root, refs) == Tree{{0, 1}, 2});
    }
    SECTION("at weight 0.2 leaves 0 and 2 pair") {
        const bvh::Reference root = REQUIRE_VALUE(hploc::build(run, nodes, hploc::Options{.error_weight = 0.2}));
        CHECK(read_tree(nodes, root, refs) == Tree{{0, 2}, 1});
    }
}

TEST_CASE("build saves the index", "[hploc][build]") {
    const std::vector<Leaf> leaves{
        {morton::Code{0}, make_unit_box_header(0.0)},
        {morton::Code{1}, make_unit_box_header(1.0)},
        {morton::Code{2}, make_unit_box_header(2.0)},
        {morton::Code{3}, make_unit_box_header(3.0)},
    };

    const TempDirectory directory("hploc");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path() / "nodes"));
    const auto [run, refs] = write_run(directory.path() / "run", nodes, {leaves});
    const size_t leaf_batches = nodes.size();

    REQUIRE_VALUE(hploc::build(run, nodes, hploc::Options{}));

    // The inner nodes of four entries fit one batch.
    const bvh::Storage reopened = REQUIRE_VALUE(bvh::Storage::open(directory.path() / "nodes"));
    CHECK(reopened.size() == leaf_batches + 1);
}
