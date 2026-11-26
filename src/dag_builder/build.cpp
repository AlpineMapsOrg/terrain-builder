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

#include <algorithm>
#include <cstdint>
#include <optional>
#include <stop_token>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <fmt/format.h>
#include <glm/gtx/component_wise.hpp>
#include <libassert/assert.hpp>
#include <tbb/concurrent_vector.h>

#include "build.h"
#include "build_config.h"
#include "centroids.h"
#include "cluster.h"
#include "clusterize.h"
#include "compact.h"
#include "geometry/geometry.h"
#include "numeric/int_math.h"
#include "cli_support/log.h"
#include "error/log.h"
#include "error/try.h"
#include "error/utils.h"
#include "fmt_impls.h"
#include "merge/clusterings.h"
#include "mesh/SimpleMesh.h"
#include "mesh/io.h"
#include "octree/Id.h"
#include "octree/IdRect.h"
#include "octree/OddLevelShifted.h"
#include "octree/Space.h"
#include "mesh/storage.h"
#include "hierarchy/traverse.h"
#include "cli_support/ProgressIndicator.h"
#include "partition.h"
#include "simplify.h"
#include "slice.h"
#include "store/Checkpointer.h"
#include "range_utils.h"
#include "storage.h"
#include "utils.h"
#include "vertex_lock.h"
#include "parallel.h"
#include "path_utils.h"
#include "store/storage_paths.h"
#include "sf/validate_index.h"

namespace dag {

namespace {

// Loads a node's mesh and clusterizes it.
alp::Result<Clustering, mesh::storage::Storage::LoadError> load_and_clusterize_mesh(
    const mesh::storage::Storage& storage,
    const octree::Id& id) {
    ALP_TRY(const mesh::Simple mesh, storage.load(id));

    LOG_DEBUG(
        "Clustering mesh for node {} with {} vertices and {} triangles",
        id,
        mesh.vertex_count(),
        mesh.face_count());

    return clusterize(mesh);
}

// A clustering with its group structure.
struct LodResult {
    Clustering clustering;
    std::vector<uint32_t> group_assignment; // per cluster group index
    PartitionToClusters group_children; // per group child indices
};

// Run the full LOD pipeline on a clustering: partition, simplify, re-clusterize, and build group structure.
LodResult build_lod(
    const Clustering &clusters,
    const BuildOptions &options,
    const radix::geometry::Aabb3d &node_bounds,
    const RegionFilter &lock_filter) {
    const Partitioning partitioning = create_partitioning(clusters, PartitionOptions{
                                                                        .clusters_per_partition = options.clusters_per_partition});
    // Apply the partitioning on geometry only.
    Clustering clustering = merge_clusters(clusters, partitioning);
    PartitionToClusters partition_to_clusters = invert_partitioning(partitioning);

    // Find vertices to lock
    const std::vector<uint8_t> vertex_lock = find_vertices_to_lock(clustering, lock_filter);

    // Convert relative target error (a fraction of the node bounds) to absolute
    const std::optional<float> absolute_target_error = options.relative_target_error.transform([&](const float relative_error) {
        return relative_error * glm::compMax(node_bounds.size());
    });
    const SimplifyOptions simplify_options{
        .target_ratio = options.target_ratio,
        .absolute_target_error = absolute_target_error,
        .vertex_lock = VertexLock::mask(vertex_lock),
        .error_mode = ErrorMode::Add,
        .preserve_cluster_count = true
    };
    clustering = simplify(clustering, simplify_options);
    remove_duplicate_triangles_inplace(clustering);

    // Unwrap the surviving geometry and render its texture.
    clustering = texture_clusters(std::move(clustering), clusters, partition_to_clusters, options.texture_options);

    // Compact the vertex buffer
    remove_unused_vertices_inplace(clustering);

    // Split each cluster into parts again.
    auto result = clusterize(clustering);

    return {std::move(result.clustering), std::move(result.backward_mapping), std::move(partition_to_clusters)};
}

// Load input meshes, clusterize, and filter them to the target region.
std::vector<Clustering> load_input_clusters(
    const std::span<const octree::Id> input_ids,
    const mesh::storage::Storage &input_storage,
    const RegionFilter& filter) {
    std::vector<Clustering> result;

    for (const octree::Id &id : input_ids) {
        auto clustering_result = load_and_clusterize_mesh(input_storage, id);
        if (!clustering_result.has_value()) {
            LOG_REPORT(clustering_result.error(), "skipping the input node");
            continue;
        }
        Clustering clustering = std::move(clustering_result).value();
        if (clustering.is_empty()) {
            LOG_WARN("Loaded clustering was empty for {}, skipping", id);
            continue;
        }
        auto indices = find_clusters_matching(clustering, filter);
        if (indices.empty()) {
            continue;
        }
        result.push_back(slice_clusters(clustering, indices));
    }

    return result;
}

// Dependencies shared across the whole dag building pipeline.
struct BuildContext {
    const mesh::storage::Storage& input_storage;
    dag::storage::Draft& output_storage;
    const BuildOptions& options;
    const octree::Space& space;
    const octree::OddLevelShifted& shifted_space;
    const radix::geometry::Aabb3d& root_bounds;
    const dag::storage::Checkpoint& checkpoint;
    mesh::storage::Draft* debug_output;
    const std::stop_token& stop;
    mutable store::Checkpointer checkpointer{};
};

// Assemble a node's metadata from a build_lod result.
dag::NodeMetadata build_node_metadata(
    std::vector<uint32_t> group_assignment,
    const PartitionToClusters &group_children,
    const Clustering &merged,
    const Clustering &simplified,
    const std::vector<dag::Id> &cluster_sources,
    const std::unordered_map<octree::Id, dag::NodeMetadata> &cluster_metadata) {
    dag::NodeMetadata metadata;
    metadata.group_assignment = std::move(group_assignment);
    metadata.triangle_counts = transform_vector(simplified.clusters, [](const Cluster &cluster) {
        return cluster.triangle_count();
    });
    metadata.groups.resize(group_children.segment_count());

    // map child indices to dag ids
    for (const auto [group_index, local_indices] : enumerate(group_children.segments())) {
        metadata.groups[group_index].children = transform_vector(local_indices, [&](const uint32_t merged_index) {
            return cluster_sources[merged.clusters[merged_index].id];
        });
    }

    // compute group bounds
    for (dag::Group &group : metadata.groups) {
        for (const dag::Id &child : group.children) {
            group.bounds.expand_by(dag::get_group_bounds(cluster_metadata.at(child.source_batch), child.cluster_index));
        }
    }

    // compute group error
    for (const auto &[final_index, group_index] : enumerate(metadata.group_assignment)) {
        double &error = metadata.groups[group_index].error;
        error = std::max(error, simplified.clusters[final_index].absolute_error);
    }

    return metadata;
}

// Load relevant DAG nodes, filter, merge, then simplify clusters.
dag::ClusterBatch load_and_simplify_dag_nodes(
    const std::vector<octree::Id> &dag_ids,
    const RegionFilter &filter,
    const double epsilon,
    const radix::geometry::Aabb3d &node_bounds,
    const BuildContext &ctx) {
    std::vector<dag::Id> cluster_sources;
    std::vector<Clustering> filtered;
    std::unordered_map<octree::Id, dag::NodeMetadata> cluster_metadata;

    for (const octree::Id &id : dag_ids) {
        // Load dag node clusters
        auto dag_node = ctx.output_storage.load(id);
        if (!dag_node) {
            LOG_REPORT(dag_node.error());
            continue;
        }
        auto &[metadata, clustering] = dag_node.value();
        cluster_metadata.emplace(id, std::move(metadata));

        // Assign canonical cluster ids
        for (auto &[cluster_index, cluster] : enumerate(clustering.clusters)) {
            cluster.id = cluster_sources.size();
            cluster_sources.emplace_back(id, cluster_index);
        }
        if (clustering.is_empty()) {
            continue;
        }

        // Find relevant clusters from this node.
        auto indices = find_clusters_matching(clustering, filter);
        if (indices.empty()) {
            continue;
        }
        filtered.push_back(slice_clusters(clustering, indices));
    }
    if (filtered.empty()) {
        return {};
    }

    // Merge remaining clusterings
    const Clustering merged = merge_clusterings(filtered, epsilon);

    // Run nanite-style grouping, simplification, splitting.
    auto [simplified, group_assignment, group_children] = build_lod(merged, ctx.options, node_bounds, filter);

    dag::NodeMetadata metadata = build_node_metadata(std::move(group_assignment), group_children, merged, simplified, cluster_sources, cluster_metadata);

    return {std::move(metadata), std::move(simplified)};
}

// Combine input clusters with inner clusters.
dag::ClusterBatch combine_input_and_inner(
    std::vector<Clustering> input_clusters,
    dag::ClusterBatch inner,
    const double epsilon) {
    const bool has_input = !input_clusters.empty();
    const bool has_inner = !inner.clustering.is_empty();
    DEBUG_ASSERT(has_input || has_inner);

    // If there are no input clusters, just return the inner clustering.
    if (!has_input) {
        return inner;
    }

    // If there are no inner clusters, just return the input clusters merged together.
    if (!has_inner) {
        Clustering merged = merge_clusterings(std::move(input_clusters), epsilon);
        return dag::make_leaf_batch(std::move(merged));
    }

    // Otherwise, merge the input clusters with the inner clustering.
    std::vector<dag::NodeMetadata> parts = transform_vector(input_clusters, [](const Clustering &input) {
        return dag::build_leaf_metadata(input);
    });
    parts.push_back(std::move(inner.metadata));
    dag::NodeMetadata metadata = concat_metadata(std::move(parts));

    input_clusters.push_back(std::move(inner.clustering));
    Clustering combined = merge_clusterings(std::move(input_clusters), epsilon);

    return {std::move(metadata), std::move(combined)};
}

// Compute epsilon value for merging clusters based on the size of the node bounds.
double compute_epsilon(const radix::geometry::Aabb3d &) {
    // only needed for .glb input
    // return glm::compAdd(bounds.size()) / 1'000'000;
    return 0.0;
}

// Build a single target node by loading input clusters (preserved as-is) and
// DAG children (inner, run through simplification pipeline), then combining both.
std::optional<dag::ClusterBatch> build_node(
    const octree::Id &target_id,
    const std::vector<octree::Id> &input_ids,
    const std::vector<octree::Id> &dag_ids,
    const BuildContext &ctx) {
    LOG_DEBUG("Building node {} ({} inputs, {} inner nodes)", target_id, input_ids.size(), dag_ids.size());

    const auto node_bounds = ctx.shifted_space.get_node_bounds(target_id);
    const double epsilon = compute_epsilon(node_bounds);

    // Prepare filter to only include clusters inside the target_id bounds.
    // In CurrentAndCoarser mode, input regions represented by the relevant DAG nodes are excluded as well.
    RegionFilter input_filter;
    input_filter.include = {node_bounds};
    if (ctx.options.include_mode == IncludeMode::CurrentAndCoarser) {
        for (const octree::Id &dag_id : dag_ids) {
            input_filter.exclude.push_back(ctx.shifted_space.get_node_bounds(dag_id));
        }
    }
    std::vector<Clustering> input_clusters = load_input_clusters(input_ids, ctx.input_storage, input_filter);

    // Mirror of input_filter: exclude the input regions so the seam vertices stay locked.
    RegionFilter dag_filter;
    dag_filter.include = {node_bounds};
    if (ctx.options.include_mode == IncludeMode::CurrentAndCoarser) {
        for (const octree::Id &input_id : input_ids) {
            dag_filter.exclude.push_back(ctx.space.get_node_bounds(input_id));
        }
    }
    dag::ClusterBatch inner = load_and_simplify_dag_nodes(dag_ids, dag_filter, epsilon, node_bounds, ctx);

    if (input_clusters.empty() && inner.clustering.is_empty()) {
        LOG_WARN("No valid clusters for node {}, skipping", target_id);
        return std::nullopt;
    }

    uint32_t input_cluster_count = 0;
    for (const auto &c : input_clusters) {
        input_cluster_count += c.cluster_count();
    }
    LOG_INFO(
        "Finished node {} with {} input + {} inner clusters",
        target_id,
        input_cluster_count,
        inner.clustering.cluster_count());

    return combine_input_and_inner(std::move(input_clusters), std::move(inner), epsilon);
}

struct LevelWorkplan {
    std::vector<octree::Id> targets;
    std::unordered_map<octree::Id, std::vector<octree::Id>> input_sources;
    std::unordered_map<octree::Id, std::vector<octree::Id>> inner_nodes;
};

// Return all input nodes from input_by_level that spatially overlap the given target node.
std::vector<octree::Id> find_relevant_input_nodes(
    const octree::Id &target_id,
    const std::vector<std::vector<octree::Id>> &input_by_level,
    const octree::OddLevelShifted &shifted_space,
    const octree::Space &space,
    const IncludeMode include_mode) {
    std::vector<octree::Id> result;
    if (include_mode == IncludeMode::CurrentOnly) {
        // Include same-level inputs that intersect with the target_id in the shifted tree.
        const std::vector<octree::Id> &input_nodes = input_by_level[target_id.level()];
        octree::IdRect standard_nodes = shifted_space.find_intersecting_standard_nodes(target_id);
        result.reserve(standard_nodes.size());
        for (const octree::Id& input_id : input_nodes) {
            if (standard_nodes.contains(input_id)) {
                result.push_back(input_id);
            }
        }
    } else if (include_mode == IncludeMode::CurrentAndCoarser) {
        // Include inputs at same or coarser levels whose bounds intersect target_id in the shifted tree.
        const radix::geometry::Aabb3d target_bounds = shifted_space.get_node_bounds(target_id);
        for (uint32_t level = 0; level <= target_id.level(); level++) {
            const std::vector<octree::Id> &input_nodes = input_by_level[level];
            for (const octree::Id& input_id : input_nodes) {
                const radix::geometry::Aabb3d input_bounds = space.get_node_bounds(input_id);
                if (radix::geometry::intersect_inclusive(target_bounds, input_bounds)) {
                    result.push_back(input_id);
                }
            }
        }
    }
    return result;
}

// The region that may hold clusters belonging to the given node. Reaches past its own bounds, since ownership is based on group centroids.
radix::geometry::Aabb3d find_cluster_search_bounds(
    const octree::Id &target_id,
    const octree::OddLevelShifted &shifted_space) {
    // We cannot rely on the intersecting nodes on the next level, since some neighbours clusters may has a centroid in this node's bounds. This can happen when the groups is split at the end of the pipeline.
    const radix::geometry::Aabb3d node_bounds = shifted_space.get_node_bounds(target_id);
    ASSERT(geometry::is_finite(node_bounds) && !geometry::is_empty(node_bounds), target_id, node_bounds);
    const radix::geometry::Aabb3d padded_bounds = geometry::pad_bounds_relative(node_bounds, 0.5 - 1e-6);
    const std::optional<radix::geometry::Aabb3d> search_bounds_opt = radix::geometry::intersection_inclusive(padded_bounds, shifted_space.bounds());
    // The padded bounds contain the node's own, which OddLevelShifted clips to the space.
    ASSERT(search_bounds_opt.has_value(), target_id);
    return *search_bounds_opt;
}

// Nodes one level finer that may hold data belonging to the given node.
octree::IdRect find_relevant_dag_nodes(
    const octree::Id &target_id,
    const octree::OddLevelShifted &shifted_space) {
    const radix::geometry::Aabb3d search_bounds = find_cluster_search_bounds(target_id, shifted_space);
    return shifted_space.get_intersecting_nodes_on_level(search_bounds, target_id.level() + 1);
}

// Nodes one level finer that may hold data belonging to the given node and are resident.
std::vector<octree::Id> find_relevant_resident_dag_nodes(
    const octree::Id &target_id,
    const std::unordered_set<octree::Id> &prev_level_built,
    const octree::OddLevelShifted &shifted_space) {
    std::vector<octree::Id> result;
    for (const octree::Id &child : find_relevant_dag_nodes(target_id, shifted_space)) {
        if (prev_level_built.contains(child)) {
            result.push_back(child);
        }
    }
    return result;
}

// Discover all shifted-space target nodes for one level from inputs and already-built children.
std::unordered_set<octree::Id> find_nodes_to_build_on_level(
    const uint32_t level,
    const std::vector<std::vector<octree::Id>> &input_by_level,
    const std::unordered_set<octree::Id> &prev_level_built,
    const BuildContext &ctx) {
    std::unordered_set<octree::Id> target_set;

    // Consider input nodes at this level
    const std::span<const octree::Id> level_input_ids = input_by_level[level];
    for (const octree::Id &input_id : level_input_ids) {
        const octree::IdRect shifted_nodes = ctx.shifted_space.find_intersecting_nodes_for_standard_id(input_id);
        for (const octree::Id &shifted_id : shifted_nodes) {
            target_set.insert(shifted_id);
        }
    }

    // Consider parents of previously built nodes
    for (const octree::Id &child : prev_level_built) {
        const octree::IdRect parents = ctx.shifted_space.get_intersecting_nodes_on_level(child, level);
        for (const octree::Id &parent : parents) {
            target_set.insert(parent);
        }
    }

    return target_set;
}

// Build a workplan for a single level, determining which target nodes to build and which input and DAG nodes are relevant for each target.
LevelWorkplan build_level_workplan(
    const uint32_t level,
    const std::vector<std::vector<octree::Id>> &input_by_level,
    const std::unordered_set<octree::Id> &prev_level_built,
    const BuildContext &ctx) {
    std::unordered_set<octree::Id> target_set = find_nodes_to_build_on_level(level, input_by_level, prev_level_built, ctx);

    std::unordered_map<octree::Id, std::vector<octree::Id>> input_sources;
    std::unordered_map<octree::Id, std::vector<octree::Id>> inner_nodes;
    for (const octree::Id &target : target_set) {
        input_sources[target] = find_relevant_input_nodes(target, input_by_level, ctx.shifted_space, ctx.space, ctx.options.include_mode);
        inner_nodes[target] = find_relevant_resident_dag_nodes(target, prev_level_built, ctx.shifted_space);
    }

    return {
        to_vector(target_set),
        std::move(input_sources),
        std::move(inner_nodes)};
}

// Pre-filter input nodes to only those that intersect the target root bounds.
std::vector<std::vector<octree::Id>> gather_relevant_input_leaves(
    const octree::storage::Index &index,
    const octree::Space &space,
    const radix::geometry::Aabb3d &root_bounds) {
    const auto start = space.find_smallest_node_encompassing_bounds(root_bounds)
        .value_or(octree::Id::root());
    std::vector<std::vector<octree::Id>> result(octree::Id::max_level() + 1);
    hierarchy::traverse(
        index.map(),
        [&](const octree::Id &id, const hierarchy::NodeStatus status) {
            if (status == hierarchy::NodeStatus::Leaf && radix::geometry::intersect_inclusive(root_bounds, space.get_node_bounds(id))) {
                result[id.level()].push_back(id);
            }
        },
        [&](const octree::Id &id) {
            return radix::geometry::intersect_inclusive(root_bounds, space.get_node_bounds(id));
        },
        start);
    return result;
}

// Find the maximum level in input_by_level that has any nodes.
std::optional<uint32_t> find_max_input_level(const std::vector<std::vector<octree::Id>> &input_by_level) {
    std::optional<uint32_t> max_level;
    for (uint32_t level = 0; level < input_by_level.size(); level++) {
        if (!input_by_level[level].empty()) {
            max_level = level;
        }
    }
    return max_level;
}

} // namespace

std::filesystem::path make_debug_mesh_path(const std::filesystem::path& output_path)
{
    auto path = paths::normalized(output_path);
    if (store::is_draft(path)) {
        path.replace_extension();
    }
    path += "-debug";
    return path;
}

// Build a single level of the DAG, returning the set of nodes that were built or the first failure.
alp::Result<std::unordered_set<octree::Id>, BuildError> build_level(
    const uint32_t level,
    const std::vector<std::vector<octree::Id>> &input_by_level,
    const std::unordered_set<octree::Id> &prev_level_built,
    const BuildContext &ctx) {
    auto [targets, input_sources, inner_nodes] =
        build_level_workplan(level, input_by_level, prev_level_built, ctx);

    if (targets.empty()) {
        return {};
    }

    LOG_INFO("Building level {} ({} targets)", level, targets.size());

    tbb::concurrent_vector<octree::Id> built_ids;

    cli_support::ProgressIndicator progress(targets.size());
    auto progress_thread = progress.start_monitoring();

    // Links the target from the checkpoint, or builds and saves it.
    const auto process_target = [&](const octree::Id& target) -> alp::Result<void, BuildError> {
        if (ctx.checkpoint.has(target)) {
            auto link_result = ctx.output_storage.link_from(target, ctx.checkpoint);
            ALP_TRY(alp::wrap_error(std::move(link_result), BuildIoError::IncompatibleCheckpoint));
            built_ids.push_back(target);
            return {};
        }

        const auto dag_ids = find_value(inner_nodes, target).value_or(std::vector<octree::Id>{});
        const auto target_input_ids = find_value(input_sources, target).value_or(std::vector<octree::Id>{});

        const std::optional<dag::ClusterBatch> batch_opt = build_node(target, target_input_ids, dag_ids, ctx);
        if (batch_opt.has_value()) {
            auto save_result = ctx.output_storage.save(target, *batch_opt);
            ALP_TRY(alp::wrap_error(std::move(save_result), BuildIoError::UnwritableOutput));
            if (ctx.debug_output != nullptr) {
                const auto debug_mesh = clustering_to_mesh(batch_opt->clustering);
                auto debug_save_result = ctx.debug_output->save(target, debug_mesh);
                ALP_TRY(alp::wrap_error(std::move(debug_save_result), BuildIoError::UnwritableOutput));
            }
            built_ids.push_back(target);
        }

        // One task at a time saves the index once the checkpoint interval has passed.
        ctx.checkpointer.save_if_due_or_warn(ctx.output_storage);
        return {};
    };

    const parallel::Execution execution = ctx.options.parallelize ? parallel::Execution::Parallel : parallel::Execution::Serial;
    // A failure stops the level like a stop request: active tasks finish their nodes, queued ones are skipped.
    auto level_result = parallel::try_for_each(targets, [&](const octree::Id& target) {
        auto target_result = process_target(target);
        progress.task_finished();
        return target_result;
    }, ctx.stop, execution);

    // Targets skipped after a failure never finish, so the monitor has to be stopped rather than waited for.
    progress_thread.request_stop();
    progress_thread.join();
    ALP_TRY(std::move(level_result));

    return std::unordered_set<octree::Id>(built_ids.begin(), built_ids.end());
}

// Builds the DAG from input_storage into output_storage, restricted to levels within level_range.
// Iterates octree levels from finest to coarsest, simplifying and re-clustering geometry at each
// level from its children.
alp::Result<void, BuildError> build_levels(
    const mesh::storage::Storage& input_storage,
    dag::storage::Draft& output_storage,
    const BuildOptions& options,
    const AnyRange<uint32_t>& level_range,
    const dag::storage::Checkpoint& checkpoint,
    mesh::storage::Draft* debug_output,
    const std::stop_token& stop) {
    auto validation_result = sf::validate_index(input_storage.index());
    ALP_TRY(alp::wrap_error(std::move(validation_result), BuildIoError::InvalidIndex));
    const octree::OddLevelShifted shifted_space = octree::OddLevelShifted::earth();
    const octree::Space space = octree::Space::earth();
    const octree::Id root_node = options.root_node;
    const auto root_bounds = shifted_space.get_node_bounds_with_children(root_node);
    const auto input_by_level = gather_relevant_input_leaves(input_storage.index(), space, root_bounds);

    // Find the maximum input level that has any nodes
    auto max_input_level_opt = find_max_input_level(input_by_level);
    if (!max_input_level_opt.has_value()) {
        LOG_WARN("No input nodes found for root {}", root_node);
        return {};
    }
    const uint32_t max_input_level = max_input_level_opt.value();

    const Range<uint32_t> valid_range{root_node.level(), max_input_level + 1};
    const Range<uint32_t> range = valid_range.intersect(level_range).to_range(max_input_level + 1);
    if (range.is_empty()) {
        LOG_WARN("Requested level range does not overlap with buildable levels {}-{}", root_node.level(), max_input_level);
        return {};
    }

    std::unordered_set<octree::Id> prev_level_built;
    BuildContext ctx{input_storage, output_storage, options, space, shifted_space, root_bounds, checkpoint, debug_output, stop};

    for (uint32_t level = range.end; level-- > range.start;) {
        auto level_context = alp::make_context("building level {}", level);

        auto level_result = build_level(level, input_by_level, prev_level_built, ctx);

        // Persist per level so a finished level can be read back before the whole run completes,
        // and after a failure so the nodes committed before it are kept
        const auto index_result = ctx.output_storage.save_index();
        LOG_WARN_IF_ERROR(index_result, "saving the index after level {}", level);
        if (ctx.debug_output != nullptr) {
            LOG_WARN_IF_ERROR(ctx.debug_output->save_index(), "saving the debug index after level {}", level);
        }
        ALP_TRY_CONTEXT(prev_level_built, std::move(level_result), level_context);
    }
    return {};
}

// Builds the complete DAG from input_storage into output_storage.
alp::Result<void, BuildError> build_full(
    const mesh::storage::Storage& input_storage,
    dag::storage::Draft& output_storage,
    const BuildOptions& options,
    const dag::storage::Checkpoint& checkpoint,
    mesh::storage::Draft* debug_output,
    const std::stop_token& stop) {
    return build_levels(input_storage, output_storage, options, RangeFull{}, checkpoint, debug_output, stop);
}

} // namespace dag
