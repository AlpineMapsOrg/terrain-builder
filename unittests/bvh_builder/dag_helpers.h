#pragma once

#include "../catch2_helpers.h"
#include "../result_helpers.h"

#include <cstddef>
#include <filesystem>
#include <optional>
#include <utility>
#include <vector>

#include "../dag_builder/dag_node.h"
#include "../dag_builder/storage.h"
#include "octree/storage/index_format.h"

// A clustering of `cluster_count` clusters that each hold one triangle.
inline Clustering make_triangle_clustering(const size_t cluster_count) {
    const Cluster triangle{.vertex_indices = {0, 1, 2}, .local_triangles = {{0, 1, 2}}, .uvs = {}, .texture_id = std::nullopt};
    return Clustering{
        .positions = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}},
        .clusters = std::vector<Cluster>(cluster_count, triangle),
    };
}

// A dag folder holding exactly the given nodes.
inline void write_dag(const std::filesystem::path &path, std::vector<std::pair<octree::Id, dag::NodeMetadata>> nodes) {
    dag::storage::Draft draft = REQUIRE_VALUE(dag::storage::create(path, octree::storage::default_layout()));
    for (auto &[id, metadata] : nodes) {
        // The codec stores a batch only with a cluster in its clustering for each one its metadata describes.
        const size_t cluster_count = metadata.clusters.size();
        REQUIRE_VALUE(draft.save(id, dag::ClusterBatch{std::move(metadata), make_triangle_clustering(cluster_count)}));
    }
    REQUIRE_VALUE(dag::storage::publish(std::move(draft)));
}
