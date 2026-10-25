#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "error/Result.h"
#include "error/try.h"
#include "io/Error.h"
#include "variant_utils.h"

#include "dag_id.h"
#include "metadata.h"
#include "dag/Cluster.h"
#include "dag/Group.h"
#include "dag/NodeMetadataCache.h"
#include "dag/SourceId.h"

namespace dag {

// Reads the cluster `id`.
// Fails if its batch cannot be read.
[[nodiscard]]
inline alp::Result<Cluster, io::Error> read_cluster(const ClusterId id, NodeMetadataCache &cache) {
    ALP_TRY(std::shared_ptr<const NodeMetadata> batch_metadata, cache.read_metadata(id.batch));
    return Cluster(id, std::move(batch_metadata));
}

// Reads the group `id`.
// Fails if its batch cannot be read.
[[nodiscard]]
inline alp::Result<Group, io::Error> read_group(const GroupId id, NodeMetadataCache &cache) {
    ALP_TRY(std::shared_ptr<const NodeMetadata> batch_metadata, cache.read_metadata(id.batch));
    return Group(id, std::move(batch_metadata));
}

// Reads the child clusters of `group`.
// Fails if one of their batches cannot be read.
[[nodiscard]]
inline alp::Result<std::vector<Cluster>, io::Error> read_clusters(const Group &group, NodeMetadataCache &cache) {
    std::vector<Cluster> result;
    result.reserve(group.cluster_ids().size());
    for (const ClusterId &id : group.cluster_ids()) {
        ALP_TRY(Cluster cluster, read_cluster(id, cache));
        result.push_back(std::move(cluster));
    }
    return result;
}

// Reads the clusters `source` stands for.
// Fails if one of their batches cannot be read.
[[nodiscard]]
inline alp::Result<std::vector<Cluster>, io::Error> read_source_clusters(const SourceId &source, NodeMetadataCache &cache) {
    return match(
        source,
        [&](const GroupId &id) -> alp::Result<std::vector<Cluster>, io::Error> {
            ALP_TRY(const Group group, read_group(id, cache));
            return read_clusters(group, cache);
        },
        [&](const ClusterId &id) -> alp::Result<std::vector<Cluster>, io::Error> {
            ALP_TRY(Cluster cluster, read_cluster(id, cache));
            return std::vector<Cluster>{std::move(cluster)};
        });
}

} // namespace dag
