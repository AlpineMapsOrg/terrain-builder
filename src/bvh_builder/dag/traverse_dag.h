#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <ranges>
#include <span>
#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <fmt/std.h>
#include <libassert/assert.hpp>
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
    std::optional<GroupId> id; // none for a root group
    double error; // its error compared to the original mesh, infinite for a root group
    radix::geometry::Aabb3d bounds; // the box its error is projected from
    std::vector<ClusterInfo> child_clusters; // the clusters it lists, which it replaces
    std::vector<ClusterInfo> parent_clusters; // the clusters it produced, which replace its child clusters

    bool operator==(const GroupInfo &) const = default;
};

// A group of the dag, borrowed for the duration of a visit.
struct GroupView {
    std::optional<GroupId> id; // none for a root group
    double error; // its error compared to the original mesh, infinite for a root group
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

// Describes the cluster `id` from `metadata`, which must be the metadata of its node.
inline ClusterInfo make_cluster_info(const Id &id, const NodeMetadata &metadata) {
    const ClusterMetadata &cluster = metadata.clusters[id.cluster_index];
    const LodMetrics metrics = get_lod_metrics_for_cluster(metadata, id.cluster_index);
    return ClusterInfo{
        .id = id,
        .error = metrics.error,
        .lod_bounds = metrics.bounds,
        .tight_bounds = cluster.bounds,
    };
}

// Counts the clusters each group of `metadata` produced.
inline std::vector<uint32_t> count_parent_clusters(const NodeMetadata &metadata) {
    std::vector<uint32_t> result(metadata.groups.size(), 0);
    for (const ClusterMetadata &cluster : metadata.clusters) {
        if (cluster.group_index.has_value()) {
            result[cluster.group_index.value()]++;
        }
    }
    return result;
}

// Collects the clusters of `node_id` by the group that produced them.
inline SegmentedBuffer<ClusterInfo, uint32_t> collect_parent_clusters(const octree::Id &node_id, const NodeMetadata &metadata) {
    SegmentedBuffer<ClusterInfo, uint32_t> result;
    result.init(count_parent_clusters(metadata));

    std::vector<uint32_t> filled(metadata.groups.size(), 0);
    for (const auto &[cluster_index, cluster] : enumerate<uint32_t>(metadata.clusters)) {
        if (!cluster.group_index.has_value()) {
            continue;
        }
        const uint32_t group_index = cluster.group_index.value();
        const uint32_t position = filled[group_index]++;
        const Id id{.source_batch = node_id, .cluster_index = cluster_index};
        result(group_index, position) = make_cluster_info(id, metadata);
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

    const auto share_node = [](const GroupChild &a, const GroupChild &b) {
        return a.cluster.source_batch == b.cluster.source_batch;
    };
    for (const auto run : group_children | std::views::chunk_by(share_node)) {
        const octree::Id node_id = run.front().cluster.source_batch;
        ALP_TRY(const std::shared_ptr<const NodeMetadata> shared_metadata, cache.read_metadata(node_id));
        const NodeMetadata &metadata = *shared_metadata;
        for (const GroupChild &child : run) {
            result(child.group_index, child.index) = make_cluster_info(child.cluster, metadata);
        }
    }
    return result;
}

// Collects the leaf clusters of `node_id`.
inline std::vector<ClusterInfo> collect_leaf_clusters(const octree::Id &node_id, const NodeMetadata &metadata) {
    std::vector<ClusterInfo> result;
    for (const auto &[cluster_index, cluster] : enumerate<uint32_t>(metadata.clusters)) {
        if (!cluster.group_index.has_value()) {
            const Id id{.source_batch = node_id, .cluster_index = cluster_index};
            result.push_back(make_cluster_info(id, metadata));
        }
    }
    return result;
}

// Views clusters no group lists as the child clusters of a root group.
inline GroupView make_root_group_view(const std::span<const ClusterInfo> clusters) {
    radix::geometry::Aabb3d bounds;
    for (const ClusterInfo &cluster : clusters) {
        bounds.expand_by(cluster.lod_bounds);
    }
    return GroupView{
        .id = std::nullopt,
        .error = std::numeric_limits<double>::infinity(),
        .bounds = bounds,
        .child_clusters = clusters,
        .parent_clusters = {},
    };
}

// Collects the nodes of `index` holding data, by global index.
inline std::vector<octree::Id> collect_nodes_with_data(const octree::IndexMap &index) {
    std::vector<octree::Id> result;
    result.reserve(index.size());
    for (const auto &[node_id, status] : index) {
        if (index.has_data(node_id)) {
            result.push_back(node_id);
        }
    }
    // Sorting by global index keeps groups that are close in space close in the walk.
    std::ranges::sort(result, {}, &octree::Id::global_index);
    return result;
}

} // namespace detail

// Visits every group of the dag, and a root group over the parent clusters of each group on the coarsest level and over each leaf cluster there.
// False if the visitor stopped the walk before its end.
template <typename F>
[[nodiscard]]
alp::Result<bool, io::Error> traverse_groups(const octree::IndexedDagMetaStorage &dag, const size_t max_resident_bytes, F &&v) {
    visitor::FallibleVisitor<io::Error, F &> visitor(v);
    NodeMetadataCache cache(dag, max_resident_bytes);

    const std::vector<octree::Id> node_ids = detail::collect_nodes_with_data(dag.index());
    if (node_ids.empty()) {
        return true;
    }
    const octree::Id::Level min_level = std::ranges::min(node_ids | std::views::transform(&octree::Id::level));

    for (const octree::Id &node_id : node_ids) {
        ALP_TRY(const std::shared_ptr<const NodeMetadata> shared_metadata, cache.read_metadata(node_id));
        const NodeMetadata &metadata = *shared_metadata;
        const bool is_coarsest = node_id.level() == min_level;
        const SegmentedBuffer<ClusterInfo, uint32_t> parent_clusters = detail::collect_parent_clusters(node_id, metadata);
        const std::vector<ClusterInfo> leaf_clusters = is_coarsest ? detail::collect_leaf_clusters(node_id, metadata) : std::vector<ClusterInfo>{};
        const std::vector<GroupMetadata> groups = metadata.groups; // intended copy
        ALP_TRY(const auto child_clusters, detail::read_child_clusters(groups, cache));

        std::vector<GroupView> group_views;
        for (const auto &[group_index, group] : enumerate<uint32_t>(groups)) {
            const GroupView group_view{
                .id = GroupId{.node = node_id, .group_index = group_index},
                .error = group.error,
                .bounds = group.bounds,
                .child_clusters = child_clusters.segment(group_index),
                .parent_clusters = parent_clusters.segment(group_index),
            };
            group_views.push_back(group_view);
        }
        // No group lists a cluster on the coarsest level, so root groups list them.
        if (is_coarsest) {
            for (const size_t group_index : range(groups.size())) {
                const std::span<const ClusterInfo> group_parent_clusters = parent_clusters.segment(group_index);
                ASSERT(!group_parent_clusters.empty(), "a group on the coarsest level has no parent clusters");
                group_views.push_back(detail::make_root_group_view(group_parent_clusters));
            }
            for (const ClusterInfo &cluster : leaf_clusters) {
                group_views.push_back(detail::make_root_group_view(std::span(&cluster, 1)));
            }
        }

        for (const GroupView &group_view : group_views) {
            ALP_TRY(const bool keep_going, visitor(group_view));
            if (!keep_going) {
                return false;
            }
        }
    }
    return true;
}

} // namespace dag

template <>
struct fmt::formatter<dag::GroupInfo> {
    constexpr auto parse(fmt::format_parse_context &ctx) {
        return ctx.begin();
    }
    auto format(const dag::GroupInfo &group, fmt::format_context &ctx) const {
        return fmt::format_to(ctx.out(), "GroupInfo(id={}, error={}, bounds={}, child_clusters={}, parent_clusters={})", group.id, group.error, group.bounds, group.child_clusters, group.parent_clusters);
    }
};
