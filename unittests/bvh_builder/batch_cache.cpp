#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "node_header_helpers.h"

#include <cstdint>
#include <memory>
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
        REQUIRE_VALUE(nodes.write(batch));
        written.push_back(std::move(batch));
    }
    REQUIRE_VALUE(nodes.save_index());
    return written;
}

}

TEST_CASE("a batch read back is the one that was written", "[storage][BatchCache]") {
    const TempDirectory directory("batch_cache");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    const std::vector<bvh::Batch> written = write_batches(nodes, 3);

    BatchCache<bvh::Storage> cache(nodes, 2 * size_bytes(written[0]));

    const std::shared_ptr<const bvh::Batch> batch = REQUIRE_VALUE(cache.read_batch(2));
    CHECK(*batch == written[2]);
}

TEST_CASE("a batch takes up the budget by its size, so a larger one displaces several smaller ones", "[storage][BatchCache]") {
    const TempDirectory directory("batch_cache");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    const std::vector<bvh::Batch> written = write_batches(nodes, 2);

    bvh::Batch large = nodes.create_batch();
    for (uint32_t group_index = 0; group_index < 3; group_index++) {
        large.add_node(make_leaf_input());
    }
    REQUIRE_VALUE(nodes.write(large));
    REQUIRE_VALUE(nodes.save_index());

    BatchCache<bvh::Storage> cache(nodes, 2 * size_bytes(written[0]));

    REQUIRE_VALUE(cache.read_batch(0));
    REQUIRE_VALUE(cache.read_batch(1));
    REQUIRE(cache.count_resident() == 2);

    REQUIRE_VALUE(cache.read_batch(large.index()));
    CHECK(cache.count_resident() == 1);
}