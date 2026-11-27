#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "node_header_helpers.h"

#include <utility>
#include <vector>

#include "bvh/utils.h"
#include "bvh/Storage.h"
#include "bvh/flat_traverse.h"

namespace {

using StoredNode = std::pair<bvh::Reference, bvh::Node>;

// `nodes_per_batch[i]` leaves in batch i, and returns every node with its reference in write order.
std::vector<StoredNode> write_nodes(bvh::Storage &nodes, const std::vector<uint32_t> &nodes_per_batch) {
    std::vector<StoredNode> written;
    for (const uint32_t count : nodes_per_batch) {
        bvh::Batch batch = nodes.create_batch();
        for (uint32_t i = 0; i < count; i++) {
            const bvh::NodeIndex node = batch.add_node(make_leaf_input());
            written.emplace_back(bvh::Reference{.batch = batch.index(), .node = node}, batch.node(node));
        }
        REQUIRE_VALUE(nodes.write(batch));
    }
    REQUIRE_VALUE(nodes.save_index());
    return written;
}

}

TEST_CASE("flat_traverse visits every node in order, with the reference naming it", "[bvh][flat_traverse]") {
    const TempDirectory directory("flat_traverse");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    const std::vector<StoredNode> written = write_nodes(nodes, {3, 1, 4});

    std::vector<StoredNode> visited;
    const auto result = bvh::flat_traverse(nodes, [&](const bvh::Reference &reference, const bvh::Node &node) {
        visited.emplace_back(reference, node);
    });

    REQUIRE(REQUIRE_VALUE(result));
    CHECK(visited == written);
}

TEST_CASE("flat_traverse handles an empty store and empty batches", "[bvh][flat_traverse]") {
    const TempDirectory directory("flat_traverse");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));

    uint32_t visits = 0;
    const auto count_visits = [&](const bvh::Reference &, const bvh::Node &) { visits++; };

    REQUIRE(REQUIRE_VALUE(bvh::flat_traverse(nodes, count_visits)));
    CHECK(visits == 0);

    write_nodes(nodes, {0, 2, 0});
    REQUIRE(REQUIRE_VALUE(bvh::flat_traverse(nodes, count_visits)));
    CHECK(visits == 2);
}

TEST_CASE("flat_traverse reports a missing batch", "[bvh][flat_traverse]") {
    const TempDirectory directory("flat_traverse");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    write_nodes(nodes, {2});

    // Reserved but never written, so reading it must fail rather than end the traversal quietly.
    nodes.create_batch();

    REQUIRE_ERROR(bvh::flat_traverse(nodes, [](const bvh::Reference &, const bvh::Node &) {}));
}

TEST_CASE("a visitor returning false stops the flat walk", "[bvh][flat_traverse]") {
    const TempDirectory directory("flat_traverse");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    write_nodes(nodes, {3, 2});

    uint32_t visits = 0;
    const auto stop_after_second = [&](const bvh::Reference &, const bvh::Node &) {
        visits++;
        return visits < 2;
    };
    const bool walked = REQUIRE_VALUE(bvh::flat_traverse(nodes, stop_after_second));

    CHECK(!walked);
    CHECK(visits == 2);
}