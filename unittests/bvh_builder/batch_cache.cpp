#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "node_header_helpers.h"

#include <cstdint>
#include <utility>
#include <vector>

#include "bvh/Storage.h"
#include "bvh/Batch.h"
#include "bvh/NodeHeader.h"
#include "storage/BatchCache.h"

namespace {

// One leaf per batch, naming the batch it landed in, and returns the batches as written.
std::vector<bvh::Batch> write_batches(bvh::Storage &nodes, const uint32_t count) {
    std::vector<bvh::Batch> written;
    for (uint32_t index = 0; index < count; index++) {
        bvh::Batch batch = nodes.create_batch();
        batch.add_node(make_leaf_input());
        REQUIRE(nodes.write(batch).has_value());
        written.push_back(std::move(batch));
    }
    REQUIRE(nodes.save_index().has_value());
    return written;
}

}

TEST_CASE("a batch read back is the one that was written", "[storage][BatchCache]") {
    const TempDirectory directory("batch_cache");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    const std::vector<bvh::Batch> written = write_batches(nodes, 3);

    BatchCache<bvh::Storage> cache(nodes, 2 * size_bytes(written[0]));

    const auto result = cache.read_batch(2);
    REQUIRE(result.has_value());
    CHECK(*result->get() == written[2]);
}

TEST_CASE("a batch takes up the budget by its size, so a larger one displaces several smaller ones", "[storage][BatchCache]") {
    const TempDirectory directory("batch_cache");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    const std::vector<bvh::Batch> written = write_batches(nodes, 2);

    bvh::Batch large = nodes.create_batch();
    for (uint32_t group_index = 0; group_index < 3; group_index++) {
        large.add_node(make_leaf_input());
    }
    REQUIRE(nodes.write(large).has_value());
    REQUIRE(nodes.save_index().has_value());

    BatchCache<bvh::Storage> cache(nodes, 2 * size_bytes(written[0]));

    REQUIRE(cache.read_batch(0).has_value());
    REQUIRE(cache.read_batch(1).has_value());
    REQUIRE(cache.count_resident() == 2);

    REQUIRE(cache.read_batch(large.index()).has_value());
    CHECK(cache.count_resident() == 1);
}