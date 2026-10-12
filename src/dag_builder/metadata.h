#pragma once

#include <cstdint>
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

struct GroupMetadata {
    std::vector<Id> children; // higher-detail clusters sharing this group's boundary
    double error; // absolute error compared to the original mesh
    radix::geometry::Aabb3d bounds; // union of the children's bounds

    bool operator==(const GroupMetadata &) const = default;
};

struct ClusterMetadata {
    uint32_t group_index; // the group of this cluster
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
    metadata.groups.reserve(clustering.cluster_count());
    for (uint32_t i = 0; i < clustering.cluster_count(); i++) {
        DEBUG_ASSERT(clustering.clusters[i].absolute_error == 0.0);
        const radix::geometry::Aabb3d bounds = compute_cluster_bounds(clustering.clusters[i], clustering.positions);
        metadata.clusters.push_back(ClusterMetadata{.group_index = i, .bounds = bounds});
        GroupMetadata &group = metadata.groups.emplace_back();
        group.error = 0.0;
        group.bounds = bounds;
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
            result.clusters.push_back(ClusterMetadata{.
                group_index = group_offset + cluster.group_index, 
                .bounds = cluster.bounds
            });
        }
        for (dag::GroupMetadata &group : part.groups) {
            result.groups.push_back(std::move(group));
        }
    }
    return result;
}

// Get the group bounds based on a cluster index.
inline const radix::geometry::Aabb3d &get_group_bounds(const NodeMetadata &metadata, const uint32_t cluster_index) {
    return metadata.groups[metadata.clusters[cluster_index].group_index].bounds;
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
