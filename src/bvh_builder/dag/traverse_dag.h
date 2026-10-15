#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <ranges>
#include <span>
#include <vector>

#include <radix/geometry.h>

#include "containers/SegmentedBuffer.h"
#include "enumerate.h"
#include "error/Result.h"
#include "error/try.h"
#include "io/Error.h"
#include "octree/Id.h"
#include "range_utils.h"

#include "metadata.h"
#include "../../dag_builder/storage.h"
#include "dag/ClusterInfo.h"
#include "dag/GroupId.h"
#include "dag/NodeMetadataCache.h"
#include "visitor.h"

namespace dag {

// A group of the dag, with the clusters it lists and the clusters it produced.
struct GroupInfo {
    GroupId id;
    double error; // its error compared to the original mesh
    radix::geometry::Aabb3d bounds; // the box its error is projected from
    std::vector<ClusterInfo> child_clusters; // the clusters it lists, which it replaces
    std::vector<ClusterInfo> parent_clusters; // the clusters it produced, which replace its child clusters

    bool operator==(const GroupInfo &) const = default;
};

// A group of the dag, with the clusters it lists and the clusters it produced.
struct GroupView {
    GroupId id;
    double error; // its error compared to the original mesh
    radix::geometry::Aabb3d bounds; // the box its error is projected from
    std::span<const ClusterInfo> child_clusters; // the clusters it lists, which it replaces
    std::span<const ClusterInfo> parent_clusters; // the clusters it produced, which replace its child clusters

    // Copies the group so it outlives the visit.
    GroupInfo to_owned() const {
        return GroupInfo{
            .id = this->id,
            .error = this->error,
            .bounds = this->bounds,
            .child_clusters = to_vector(this->child_clusters),
            .parent_clusters = to_vector(this->parent_clusters),
        };
    }
};

namespace detail {

// Counts the clusters each group of `metadata` produced.
inline std::vector<uint32_t> count_parent_clusters(const NodeMetadata &metadata) {
    std::vector<uint32_t> result(metadata.groups.size(), 0);
    for (const ClusterMetadata &cluster : metadata.clusters) {
        result[cluster.group_index]++;
    }
    return result;
}

// Collects the clusters of `node_id` by the group that produced them.
inline SegmentedBuffer<ClusterInfo, uint32_t> collect_parent_clusters(const octree::Id &node_id, const NodeMetadata &metadata) {
    SegmentedBuffer<ClusterInfo, uint32_t> result;
    result.init(count_parent_clusters(metadata));

    std::vector<uint32_t> filled(metadata.groups.size(), 0);
    for (const auto &[cluster_index, cluster] : enumerate<uint32_t>(metadata.clusters)) {
        const uint32_t position = filled[cluster.group_index]++;
        const GroupMetadata &group = metadata.groups[cluster.group_index];
        result(cluster.group_index, position) = ClusterInfo{
            .id = Id{.source_batch = node_id, .cluster_index = cluster_index},
            .error = group.error,
            .lod_bounds = group.bounds,
            .tight_bounds = cluster.bounds,
        };
    }
    return result;
}

// A cluster a group lists.
struct GroupChild {
    uint32_t group_index; // the group listing it
    uint32_t index; // its position among the clusters the group lists
    Id cluster;
};

// Reads the clusters each of `groups` lists, by group.
[[nodiscard]]
inline alp::Result<SegmentedBuffer<ClusterInfo, uint32_t>, io::Error> read_child_clusters(std::span<const GroupMetadata> groups, NodeMetadataCache &cache) {
    // Sorted so that the clusters of one node come together and share its read.
    std::vector<GroupChild> group_children;
    for (const auto &[group_index, group] : enumerate<uint32_t>(groups)) {
        for (const auto &[index, id] : enumerate<uint32_t>(group.children)) {
            group_children.push_back(GroupChild{.group_index = group_index, .index = index, .cluster = id});
        }
    }
    std::ranges::sort(group_children, {}, &GroupChild::cluster);

    const std::vector<uint32_t> counts = transform_vector(groups, [](const GroupMetadata &group) {
        return static_cast<uint32_t>(group.children.size());
    });
    SegmentedBuffer<ClusterInfo, uint32_t> result;
    result.init(counts);

    const auto share_node = [](const GroupChild &a, const GroupChild &b) { return a.cluster.source_batch == b.cluster.source_batch; };
    for (const auto run : group_children | std::views::chunk_by(share_node)) {
        const octree::Id node_id = run.front().cluster.source_batch;
        ALP_TRY(const NodeMetadata &metadata, cache.read_metadata(node_id));
        for (const GroupChild &child : run) {
            const ClusterMetadata &cluster = metadata.clusters[child.cluster.cluster_index];
            const GroupMetadata &group = metadata.groups[cluster.group_index];
            result(child.group_index, child.index) = ClusterInfo{
                .id = child.cluster,
                .error = group.error,
                .lod_bounds = group.bounds,
                .tight_bounds = cluster.bounds,
            };
        }
    }
    return result;
}

} // namespace detail

// Visits every group of the dag.
// False if the visitor stopped the walk before its end.
template <typename F>
[[nodiscard]]
alp::Result<bool, io::Error> traverse_groups(const octree::IndexedDagMetaStorage &dag, const size_t max_resident_bytes, F &&v) {
    visitor::FallibleVisitor<io::Error, F &> visitor(v);
    NodeMetadataCache cache(dag, max_resident_bytes);

    const octree::IndexMap &index = dag.index();
    std::vector<octree::Id> node_ids;
    node_ids.reserve(index.size());
    for (const auto &[node_id, status] : index) {
        if (index.has_data(node_id)) {
            node_ids.push_back(node_id);
        }
    }
    // Sorting by global index keeps groups that are close in space close in the walk.
    std::ranges::sort(node_ids, {}, &octree::Id::global_index);

    for (const octree::Id &node_id : node_ids) {
        ALP_TRY(const NodeMetadata &metadata, cache.read_metadata(node_id));
        const std::vector<GroupMetadata> groups = metadata.groups;
        const SegmentedBuffer<ClusterInfo, uint32_t> parent_clusters = detail::collect_parent_clusters(node_id, metadata);
        ALP_TRY(const auto child_clusters, detail::read_child_clusters(groups, cache));

        for (const auto &[group_index, group] : enumerate<uint32_t>(groups)) {
            const GroupView group_view{
                .id = GroupId{.node = node_id, .group_index = group_index},
                .error = group.error,
                .bounds = group.bounds,
                .child_clusters = child_clusters.segment(group_index),
                .parent_clusters = parent_clusters.segment(group_index),
            };
            ALP_TRY(const bool keep_going, visitor(group_view));
            if (!keep_going) {
                return false;
            }
        }
    }

    // We walked to the end
    return true;
}

} // namespace dag
