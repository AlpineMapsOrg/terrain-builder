#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include <libassert/assert.hpp>
#include <radix/geometry.h>

#include "cluster.h"
#include "dag_id.h"
#include "utils.h"
#include "zpp_bits_utils.h"
#include "octree/storage/defaults.h"
#include "octree/storage/codec/ZppBitsCodec.h"
#include "octree/storage/codec/ReadOnlyCodec.h"


namespace dag {

struct LodMetrics {
    radix::geometry::Aabb3d bounds;
    double error;

    bool operator==(const LodMetrics &) const = default;
};

struct GroupMetadata {
    std::vector<Id> children = {}; // its child clusters, the higher-detail clusters it replaces
    double error; // absolute error compared to the original mesh
    radix::geometry::Aabb3d bounds; // union of its child clusters' lod bounds

    bool operator==(const GroupMetadata &) const = default;
};

struct ClusterMetadata {
    std::optional<uint32_t> group_index; // the group this is a parent cluster of, none for a leaf cluster
    radix::geometry::Aabb3d bounds; // the cluster's own bounds

    bool operator==(const ClusterMetadata &) const = default;
};

struct NodeMetadata {
    std::vector<ClusterMetadata> clusters; // by cluster index
    std::vector<GroupMetadata> groups;

    uint32_t group_count() const {
        return this->groups.size();
    }
};

inline NodeMetadata build_leaf_metadata(const Clustering &clustering) {
    NodeMetadata metadata;
    metadata.clusters.reserve(clustering.cluster_count());
    for (const Cluster &cluster : clustering.clusters) {
        DEBUG_ASSERT(cluster.absolute_error == 0.0);
        const radix::geometry::Aabb3d bounds = compute_cluster_bounds(cluster, clustering.positions);
        metadata.clusters.push_back(ClusterMetadata{
            .group_index = std::nullopt, 
            .bounds = bounds
        });
    }
    return metadata;
}

// Concatenate metadatas in order.
inline dag::NodeMetadata concat_metadata(std::vector<dag::NodeMetadata> parts) {
    const auto total_clusters = sum(parts, [](const dag::NodeMetadata &part) { return part.clusters.size(); });
    const auto total_groups = sum(parts, [](const dag::NodeMetadata &part) { return part.groups.size(); });

    dag::NodeMetadata result;
    result.clusters.reserve(total_clusters);
    result.groups.reserve(total_groups);

    for (dag::NodeMetadata &part : parts) {
        const uint32_t group_offset = result.group_count();
        for (const ClusterMetadata &cluster : part.clusters) {
            result.clusters.push_back(ClusterMetadata{
                .group_index = cluster.group_index.transform([&](const uint32_t index) { return group_offset + index; }),
                .bounds = cluster.bounds
            });
        }

        for (dag::GroupMetadata &group : part.groups) {
            result.groups.push_back(std::move(group));
        }
    }
    return result;
}

// The lod metrics of the cluster at `cluster_index`: the error and bounds of the group it is a parent cluster of, or its own bounds with error 0 for a leaf cluster.
[[nodiscard]] inline LodMetrics get_lod_metrics_for_cluster(const NodeMetadata &metadata, const uint32_t cluster_index) {
    const ClusterMetadata &cluster = metadata.clusters[cluster_index];
    if (!cluster.group_index.has_value()) {
        return LodMetrics{
            .bounds = cluster.bounds,
            .error = 0.0
        };
    }

    const GroupMetadata &group = metadata.groups[cluster.group_index.value()];
    return LodMetrics{
        .bounds = group.bounds, 
        .error = group.error
    };
}

} // namespace dag

// Set default codec for metadata.
namespace octree {
template <>
struct DefaultCodec<dag::NodeMetadata> {
    using type = ReadOnlyCodec<ZppBitsCodec<dag::NodeMetadata>>;
};
}

namespace zpp::bits {

template <typename Archive>
auto serialize(Archive &archive, const dag::GroupMetadata &group) {
    return archive(group.children, group.error, group.bounds);
}
template <typename Archive>
auto serialize(Archive &archive, dag::GroupMetadata &group) {
    return archive(group.children, group.error, group.bounds);
}

template <typename Archive>
auto serialize(Archive &archive, const dag::ClusterMetadata &cluster) {
    return archive(cluster.group_index, cluster.bounds);
}
template <typename Archive>
auto serialize(Archive &archive, dag::ClusterMetadata &cluster) {
    return archive(cluster.group_index, cluster.bounds);
}

template <typename Archive>
auto serialize(Archive &archive, const dag::NodeMetadata &metadata) {
    return archive(metadata.clusters, metadata.groups);
}
template <typename Archive>
auto serialize(Archive &archive, dag::NodeMetadata &metadata) {
    return archive(metadata.clusters, metadata.groups);
}

} // namespace zpp::bits
