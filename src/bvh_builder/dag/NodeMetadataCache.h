#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

#include "error/Result.h"
#include "error/try.h"
#include "io/Error.h"
#include "octree/Id.h"

#include "metadata.h"
#include "../../dag_builder/storage.h"
#include "storage/BoundedCache.h"

namespace dag {

// A cache of dag node metadata, bounded by its size in bytes.
class NodeMetadataCache {
public:
    NodeMetadataCache(const octree::IndexedDagMetaStorage &dag, const size_t max_resident_bytes)
        : _dag(dag), _cache(max_resident_bytes) {}

    // Reads the metadata of `batch`.
    [[nodiscard]]
    alp::Result<std::shared_ptr<const NodeMetadata>, io::Error> read_metadata(const BatchId &batch) {
        return this->_cache.get_or_try_insert_with(batch, [&]() -> alp::Result<Cache::Entry, io::Error> {
            ALP_TRY(NodeMetadata metadata, this->_dag.load(batch));
            const size_t cost = estimate_bytes(metadata);
            std::shared_ptr<const NodeMetadata> shared_metadata = std::make_shared<const NodeMetadata>(std::move(metadata));

            return Cache::Entry{.value = std::move(shared_metadata), .cost = cost};
        });
    }

    // How many reads had to load a node.
    uint32_t count_misses() const {
        return this->_cache.count_misses();
    }

private:
    using Cache = BoundedCache<BatchId, std::shared_ptr<const NodeMetadata>>;

    // Estimates how many bytes `metadata` holds.
    static size_t estimate_bytes(const NodeMetadata &metadata) {
        size_t bytes = sizeof(NodeMetadata) + metadata.clusters.capacity() * sizeof(ClusterMetadata) + metadata.groups.capacity() * sizeof(GroupMetadata);
        for (const GroupMetadata &group : metadata.groups) {
            bytes += group.children.capacity() * sizeof(ClusterId);
        }
        return bytes;
    }

    const octree::IndexedDagMetaStorage &_dag;
    Cache _cache;
};

} // namespace dag
