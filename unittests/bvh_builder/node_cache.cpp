#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "node_header_helpers.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "bvh/NodeCache.h"
#include "bvh/Storage.h"
#include "bvh/Batch.h"
#include "bvh/NodeHeader.h"
#include "bvh/Reference.h"

namespace {

constexpr size_t UNBOUNDED_BYTES = std::numeric_limits<size_t>::max();

// `nodes_per_batch[i]` leaves in batch i, and returns the batches as written.
std::vector<bvh::Batch> write_leaves(bvh::Storage &nodes, const std::vector<uint32_t> &nodes_per_batch) {
    std::vector<bvh::Batch> written;
    uint32_t group_index = 0;
    for (const uint32_t count : nodes_per_batch) {
        bvh::Batch batch = nodes.create_batch();
        for (uint32_t i = 0; i < count; i++) {
            batch.add_node(make_leaf_input());
            group_index++;
        }
        REQUIRE(nodes.write(batch).has_value());
        written.push_back(std::move(batch));
    }
    REQUIRE(nodes.save_index().has_value());
    return written;
}

}

TEST_CASE("a node comes back from the batch it was written to", "[bvh][NodeCache]") {
    const TempDirectory directory("node_cache");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    const std::vector<bvh::Batch> written = write_leaves(nodes, {2, 3});

    bvh::NodeCache cache(nodes, UNBOUNDED_BYTES);

    const auto node = cache.read_node(bvh::Reference{.batch = 1, .node = 2});
    REQUIRE(node.has_value());
    CHECK(*node == written[1].node(2));
}

TEST_CASE("nodes in the same batch are read once", "[bvh][NodeCache]") {
    const TempDirectory directory("node_cache");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    write_leaves(nodes, {2, 3});

    bvh::NodeCache cache(nodes, UNBOUNDED_BYTES);

    REQUIRE(cache.read_node(bvh::Reference{.batch = 1, .node = 0}).has_value());
    REQUIRE(cache.read_node(bvh::Reference{.batch = 1, .node = 1}).has_value());
    REQUIRE(cache.read_node(bvh::Reference{.batch = 1, .node = 2}).has_value());

    CHECK(cache.count_misses() == 1);
}

TEST_CASE("a node outlives the read that follows it", "[bvh][NodeCache]") {
    const TempDirectory directory("node_cache");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    const std::vector<bvh::Batch> written = write_leaves(nodes, {1, 1, 1});

    bvh::NodeCache cache(nodes, 1);

    const auto first = cache.read_node(bvh::Reference{.batch = 0, .node = 0});
    REQUIRE(first.has_value());
    REQUIRE(cache.read_node(bvh::Reference{.batch = 2, .node = 0}).has_value());

    CHECK(*first == written[0].node(0));
}

TEST_CASE("the batch behind a node is reachable through the cache", "[bvh][NodeCache]") {
    const TempDirectory directory("node_cache");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    const std::vector<bvh::Batch> written = write_leaves(nodes, {2, 3});

    bvh::NodeCache cache(nodes, UNBOUNDED_BYTES);

    const auto result = cache.read_batch(1);
    REQUIRE(result.has_value());
    CHECK(*result->get() == written[1]);
}