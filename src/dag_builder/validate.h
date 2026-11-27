/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2025 Martin Braunsperger
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

#include <cmath>
#include <cstdint>
#include <source_location>
#include <span>
#include <utility>

#include <fmt/format.h>
#include <glm/glm.hpp>

#include "cluster.h"
#include "enumerate.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "error/unwrap.h"
#include "error/utils.h"
#include "mesh/validate/result.h"
#include "utils.h"

struct InconsistentCluster : alp::MessageCode<"the cluster is inconsistent"> {};
struct InconsistentClustering : alp::MessageCode<"the clustering is inconsistent"> {};

[[nodiscard]]
inline alp::Result<void, InconsistentCluster> validate(const Cluster &cluster, const std::span<const glm::dvec3> positions) {
    const size_t cluster_vertex_count = cluster.vertex_indices.size();
    const size_t mesh_vertex_count = positions.size();

    // Cluster must have vertices
    if (cluster.vertex_indices.empty()) {
        return alp::make_error(InconsistentCluster{}, "the cluster has no vertices");
    }

    // Cluster must have triangles
    if (cluster.local_triangles.empty()) {
        return alp::make_error(InconsistentCluster{}, "the cluster has no triangles");
    }

    // Cluster must have one UV per vertex if it has UVs
    if (cluster.has_uvs() && cluster.uvs.size() != cluster_vertex_count) {
        return alp::make_error(InconsistentCluster{}, fmt::format("the cluster has {} UVs for {} vertices", cluster.uvs.size(), cluster_vertex_count));
    }

    // Cluster must have UVs exactly when it has a texture
    if (cluster.has_uvs() && !cluster.is_textured()) {
        return alp::make_error(InconsistentCluster{}, "the cluster has UVs but no texture");
    }
    if (cluster.is_textured() && !cluster.has_uvs()) {
        return alp::make_error(InconsistentCluster{}, "the cluster has a texture but no UVs");
    }

    // Cluster error must be finite
    if (!std::isfinite(cluster.absolute_error)) {
        return alp::make_error(InconsistentCluster{}, "the cluster has a non-finite error");
    }

    // Validate vertex indices reference valid mesh vertices
    for (const auto [local_vertex_index, vertex_index] : enumerate(cluster.vertex_indices)) {
        if (vertex_index >= mesh_vertex_count) {
            return alp::make_error(InconsistentCluster{}, fmt::format("vertex {} names position {}, but there are {} positions", local_vertex_index, vertex_index, mesh_vertex_count));
        }
    }

    // Validate triangle indices reference cluster-local vertices
    for (const auto [triangle_index, triangle] : enumerate(cluster.local_triangles)) {
        for (uint8_t corner = 0; corner < 3; corner++) {
            if (triangle[corner] >= cluster_vertex_count) {
                return alp::make_error(InconsistentCluster{}, fmt::format("triangle {} names vertex {}, but the cluster has {} vertices", triangle_index, triangle[corner], cluster_vertex_count));
            }
        }
    }

    // Materialize cluster mesh to validate mesh
    const mesh::Simple mesh = materialize_cluster(cluster, positions);
    auto mesh_result = mesh::validate_basic(mesh);
    if (!mesh_result.has_value()) {
        return alp::make_error(InconsistentCluster{}, std::move(mesh_result).error());
    }

    return {};
}

[[nodiscard]]
inline alp::Result<void, InconsistentClustering> validate(const Clustering &clustering) {
    for (const auto [cluster_index, cluster] : enumerate(clustering.clusters)) {
        // Cluster must be consistent on its own
        auto cluster_result = validate(cluster, clustering.positions);
        if (!cluster_result.has_value()) {
            return alp::make_error(InconsistentClustering{}, fmt::format("checking cluster {}", cluster_index), std::move(cluster_result).error());
        }

        // Cluster texture must be in the clustering's texture set
        if (cluster.is_textured() && !clustering.textures.contains_id(cluster.texture_id.value())) {
            return alp::make_error(InconsistentClustering{}, fmt::format("cluster {} names texture {}, but the clustering holds no such texture", cluster_index, cluster.texture_id.value()));
        }
    }

    return {};
}

inline void assert_valid(const Cluster &cluster, const std::span<const glm::dvec3> positions, const std::source_location location = std::source_location::current()) {
    alp::unwrap(validate(cluster, positions), location);
}

inline void assert_valid(const Clustering &clustering, const std::source_location location = std::source_location::current()) {
    alp::unwrap(validate(clustering), location);
}
