#pragma once

#include <cstddef>
#include <expected>
#include <utility>
#include <vector>

#include <libassert/assert.hpp>
#include <radix/geometry.h>

#include "error/Result.h"
#include "error/try.h"
#include "io/Error.h"
#include "range_utils.h"

#include "../../dag_builder/storage.h"

#include "bvh/NodeHeader.h"
#include "bvh/NodeInput.h"
#include "bvh/Storage.h"
#include "bvh/BatchWriter.h"
#include "dag/Cluster.h"
#include "dag/ClusterInfo.h"
#include "dag/Group.h"
#include "dag/NodeMetadataCache.h"
#include "dag/read.h"
#include "dag/visit_dag.h"

namespace bvh {

struct FlattenOptions {
    size_t max_batch_bytes = DEFAULT_MAX_BATCH_BYTES; // the most bytes a written batch holds
    size_t max_resident_bytes = DEFAULT_MAX_RESIDENT_BYTES; // the most bytes of dag node metadata kept cached
};

namespace detail {

// Copies `cluster` into the record a leaf holds.
inline dag::ClusterInfo make_cluster_info(const dag::Cluster &cluster) {
    return dag::ClusterInfo{.id = cluster.id(), .error = cluster.error(), .lod_bounds = cluster.lod_bounds(), .tight_bounds = cluster.tight_bounds()};
}

// Makes the leaf holding the child clusters of `group`.
[[nodiscard]]
inline alp::Result<LeafInput<std::vector<dag::ClusterInfo>>, io::Error> make_group_leaf(const dag::Group &group, dag::NodeMetadataCache &cache) {
    ALP_TRY(const std::vector<dag::Cluster> clusters, dag::read_clusters(group, cache));
    DEBUG_ASSERT(!clusters.empty());

    radix::geometry::Aabb3d tight_bounds;
    for (const dag::Cluster &cluster : clusters) {
        tight_bounds.expand_by(cluster.tight_bounds());
    }

    const double min_child_error = min(clusters, &dag::Cluster::error);
    const NodeHeader header{
        .min_child_error = min_child_error,
        .max_parent_error = group.error(),
        .lod_bounds = group.lod_bounds(),
        .tight_bounds = tight_bounds,
    };
    std::vector<dag::ClusterInfo> cluster_infos = transform_vector(clusters, make_cluster_info);
    return LeafInput<std::vector<dag::ClusterInfo>>{header, std::move(cluster_infos)};
}

// Makes the leaf holding `cluster`, which no group lists.
inline LeafInput<std::vector<dag::ClusterInfo>> make_cluster_leaf(const dag::Cluster &cluster) {
    const NodeHeader header{
        .min_child_error = cluster.error(),
        .max_parent_error = ROOT_PARENT_ERROR,
        .lod_bounds = cluster.lod_bounds(),
        .tight_bounds = cluster.tight_bounds(),
    };
    std::vector<dag::ClusterInfo> cluster_infos{make_cluster_info(cluster)};
    return LeafInput<std::vector<dag::ClusterInfo>>{header, std::move(cluster_infos)};
}

} // namespace detail

// Writes the leaves of a bvh over `dag` into `output`.
[[nodiscard]]
inline alp::Result<void, io::Error> flatten_dag(const octree::IndexedDagMetaStorage &dag, Storage &output, const FlattenOptions &options = {}) {
    BatchWriter writer(output, options.max_batch_bytes);
    dag::NodeMetadataCache cache(dag, options.max_resident_bytes);

    ALP_TRY(dag::visit_groups([&](const dag::Group &group) -> alp::Result<void, io::Error> {
        ALP_TRY(const LeafInput<std::vector<dag::ClusterInfo>> leaf, detail::make_group_leaf(group, cache));
        ALP_TRY(writer.write_node(leaf));
        return {};
    }, cache));

    ALP_TRY(dag::visit_root_clusters([&](const dag::Cluster &cluster) -> alp::Result<void, io::Error> {
        ALP_TRY(writer.write_node(detail::make_cluster_leaf(cluster)));
        return {};
    }, cache));

    return writer.flush();
}

}
