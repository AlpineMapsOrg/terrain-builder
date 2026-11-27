/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Martin Braunsperger
 * Copyright (C) 2026 Adam Celarek-Litofcenko
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include <libassert/assert.hpp>
#include <radix/geometry.h>

#include "cluster.h"
#include "dag_id.h"
#include "utils.h"

namespace dag {

struct LodMetrics {
    radix::geometry::Aabb3d bounds;
    double error;

    bool operator==(const LodMetrics &) const = default;
};

struct GroupMetadata {
    std::vector<ClusterId> children = {}; // its child clusters, the higher-detail clusters it replaces
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

    bool operator==(const NodeMetadata &) const = default;

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
