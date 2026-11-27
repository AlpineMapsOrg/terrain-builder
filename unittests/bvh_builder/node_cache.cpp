#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "node_header_helpers.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
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
    for (const uint32_t count : nodes_per_batch) {
        bvh::Batch batch = nodes.create_batch();
        for (uint32_t i = 0; i < count; i++) {
            batch.add_node(make_leaf_input());
        }
        REQUIRE_VALUE(nodes.write(batch));
        written.push_back(std::move(batch));
    }
    REQUIRE_VALUE(nodes.save_index());
    return written;
}

}

TEST_CASE("a node comes back from the batch it was written to", "[bvh][NodeCache]") {
    const TempDirectory directory("node_cache");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    const std::vector<bvh::Batch> written = write_leaves(nodes, {2, 3});

    bvh::NodeCache cache(nodes, UNBOUNDED_BYTES);

    const bvh::Node node = REQUIRE_VALUE(cache.read_node(bvh::Reference{.batch = 1, .node = 2}));
    CHECK(node == written[1].node(2));
}

TEST_CASE("nodes in the same batch are read once", "[bvh][NodeCache]") {
    const TempDirectory directory("node_cache");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    write_leaves(nodes, {2, 3});

    bvh::NodeCache cache(nodes, UNBOUNDED_BYTES);

    REQUIRE_VALUE(cache.read_node(bvh::Reference{.batch = 1, .node = 0}));
    REQUIRE_VALUE(cache.read_node(bvh::Reference{.batch = 1, .node = 1}));
    REQUIRE_VALUE(cache.read_node(bvh::Reference{.batch = 1, .node = 2}));

    CHECK(cache.count_misses() == 1);
}

TEST_CASE("a node outlives the read that follows it", "[bvh][NodeCache]") {
    const TempDirectory directory("node_cache");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    const std::vector<bvh::Batch> written = write_leaves(nodes, {1, 1, 1});

    bvh::NodeCache cache(nodes, 1);

    const bvh::Node first = REQUIRE_VALUE(cache.read_node(bvh::Reference{.batch = 0, .node = 0}));
    REQUIRE_VALUE(cache.read_node(bvh::Reference{.batch = 2, .node = 0}));

    CHECK(first == written[0].node(0));
}

TEST_CASE("the batch behind a node is reachable through the cache", "[bvh][NodeCache]") {
    const TempDirectory directory("node_cache");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    const std::vector<bvh::Batch> written = write_leaves(nodes, {2, 3});

    bvh::NodeCache cache(nodes, UNBOUNDED_BYTES);

    const std::shared_ptr<const bvh::Batch> batch = REQUIRE_VALUE(cache.read_batch(1));
    CHECK(*batch == written[1]);
}