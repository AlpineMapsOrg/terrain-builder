/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2024 Martin Braunsperger
 * Copyright (C) 2024 Adam Celarek-Litofcenko
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

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fmt/core.h>
#include <glm/glm.hpp>
#include <libassert/assert.hpp>
#include <ogr_spatialref.h>
#include <opencv2/core/mat.hpp>
#include <radix/geometry.h>

#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/task_group.h>

#include "gis/Dataset.h"
#include "gis/srs.h"

#include "cli_support/ProgressIndicator.h"
#include "cli_support/log.h"
#include "mesh/SimpleMesh.h"
#include "mesh/io.h"
#include "mesh_builder.h"
#include "terrainbuilder.h"
#include "texture_assembler.h"
#include "tile_provider.h"
#include "mesh/validate.h"

#include "octree/Id.h"
#include "octree/Space.h"
#include "octree/utils.h"
#include "sf/finalize_storage.h"
#include "store/ThreadSafeStorage.h"
#include "store/inputs.h"
#include "store/snapshot.h"

namespace terrainbuilder {

namespace {
std::string format_secs_since(const std::chrono::high_resolution_clock::time_point &start) {
    const auto duration = std::chrono::high_resolution_clock::now() - start;
    const double seconds = std::chrono::duration<double>(duration).count();
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << seconds;
    return ss.str();
}
}

std::optional<SimpleMesh> build_patch(
    gis::Dataset &dataset,
    const OGRSpatialReference &target_bounds_srs,
    const radix::geometry::Aabb3d &target_bounds,
    const OGRSpatialReference &texture_srs,
    const TileProvider* tile_provider,
    const OGRSpatialReference &mesh_srs) {
    radix::tile::SrsBounds texture_bounds;

    std::chrono::high_resolution_clock::time_point start;
    start = std::chrono::high_resolution_clock::now();
    LOG_INFO("Building mesh...");
    std::expected<SimpleMesh, BuildMeshError> mesh_result = build_reference_mesh_patch(
        dataset,
        mesh_srs,
        target_bounds_srs, target_bounds,
        texture_srs, texture_bounds);
    if (!mesh_result) {
        const BuildMeshError error = mesh_result.error();
        if (error == BuildMeshError::OutOfBounds) {
            const auto dataset_bounds_result = dataset.bounds();
            if (!dataset_bounds_result) {
                LOG_ERROR("Target bounds are fully outside of dataset region: {}", dataset_bounds_result.error().to_string());
                return std::nullopt;
            }
            const radix::tile::SrsBounds dataset_bounds = *dataset_bounds_result;
            LOG_ERROR("Target bounds are fully outside of dataset region\n"
                      "Dataset {{\n"
                      "\t x={}, y={}, w={}, h={}.\n"
                      "}}\n"
                      "Target {{\n"
                      "\t x={}, y={}, w={}, h={}.\n"
                      "}}",
                      dataset_bounds.min.x, dataset_bounds.min.y, dataset_bounds.width(), dataset_bounds.height(),
                      target_bounds.min.x, target_bounds.min.y, target_bounds.size().x, target_bounds.size().y);
            return std::nullopt;
        } else if (error == BuildMeshError::EmptyRegion) {
            LOG_WARN("Target bounds are inside dataset, but the region is empty");
            return std::nullopt;
        }
        // Transformation failures are logged where they occur.
        return std::nullopt;
    }
    SimpleMesh mesh = mesh_result.value();
    LOG_DEBUG("Mesh building took {}s", format_secs_since(start));
    LOG_INFO("Finished building mesh geometry");

    if (tile_provider != nullptr) {
        start = std::chrono::high_resolution_clock::now();
        LOG_INFO("Assembling mesh texture");
        std::optional<AssembledTexture> texture = assemble_texture_from_tiles(texture_srs, texture_bounds, *tile_provider);
        if (!texture.has_value()) {
            LOG_ERROR("Failed to assemble texture");
            // TODO: should we return nullopt here?
        } else {
            // The uvs address the requested bounds, the texture holds padding around them.
            texture.value().remap_uvs(mesh.uvs);
            mesh.texture = texture.value().image;
        }
        LOG_DEBUG("Assembling mesh texture took {}s", format_secs_since(start));
        LOG_INFO("Finished assembling mesh texture");
    } else {
        LOG_INFO("Skipped assembling texture");
    }
    
    return mesh;
}

void build_and_save_patch(
    gis::Dataset &dataset,
    const OGRSpatialReference &target_bounds_srs,
    const radix::geometry::Aabb3d &target_bounds,
    const OGRSpatialReference &texture_srs,
    const TileProvider *tile_provider,
    const OGRSpatialReference &mesh_srs,
    const std::filesystem::path &output_path) {
    auto mesh_result = build_patch(
        dataset,
        target_bounds_srs,
        target_bounds,
        texture_srs,
        tile_provider,
        mesh_srs);
    if (!mesh_result.has_value()) {
        return;
    }
    const SimpleMesh mesh = std::move(mesh_result.value());

    LOG_INFO("Writing mesh to output path {}", output_path);

    std::chrono::high_resolution_clock::time_point start;
    start = std::chrono::high_resolution_clock::now();
    // TODO: use a JSON libary instead
    std::unordered_map<std::string, std::string> metadata;
    metadata["mesh_srs"] = mesh_srs.GetAuthorityCode(nullptr);
    metadata["bounds_srs"] = target_bounds_srs.GetAuthorityCode(nullptr);
    metadata["texture_srs"] = texture_srs.GetAuthorityCode(nullptr);
    metadata["bounds"] = fmt::format(
        "{{ \"min\": {{ \"x\": {}, \"y\": {} }}, \"max\": {{ \"x\": {}, \"y\": {} }} }}",
        target_bounds.min.x, target_bounds.min.y, target_bounds.max.x, target_bounds.max.y);
    // metadata["texture_bounds"] = fmt::format(
    //     "{{ \"min\": {{ \"x\": {}, \"y\": {} }}, \"max\": {{ \"x\": {}, \"y\": {} }} }}",
    //    texture_bounds.min.x, texture_bounds.min.y, texture_bounds.max.x, texture_bounds.max.y);
    const auto saved = mesh::io::save_to_path(mesh, output_path, mesh::io::SaveOptions{.metadata = metadata});
    if (!saved) {
        LOG_ERROR("Failed to save mesh to file {}: {}", output_path, saved.error().to_string());
        std::exit(2);
    }
    LOG_DEBUG("Writing mesh took {}s", format_secs_since(start));
    LOG_INFO("Done writing mesh to {}", output_path);
}

namespace {
template <typename T>
T expect(const std::optional<T> &opt, const std::string &msg) {
    if (!opt) {
        LOG_ERROR_AND_EXIT(msg);
    }
    return *opt;
}
}

Expected<void> build_all_patches(gis::Dataset& dataset,
    const OGRSpatialReference& texture_srs,
    const TileProvider* tile_provider,
    const OGRSpatialReference& mesh_srs,
    const std::filesystem::path& output_path,
    const inputs::Record& record,
    const std::optional<std::filesystem::path>& cache_path,
    const std::function<bool()>& stop_requested)
{
    if (record.target_level < min_target_level || record.target_level > octree::Id::max_level()) {
        return Error::fail(Error::Code::InvalidInput,
            fmt::format("target level {} is outside of {}..{}", record.target_level, unsigned(min_target_level), unsigned(octree::Id::max_level())));
    }
    const auto target_level = octree::Id::Level(record.target_level);

    std::optional<mesh::storage::IndexedStorage> cache;
    if (cache_path.has_value()) {
        if (auto valid = store::inputs::validate_cache<inputs::Schema>(*cache_path, record); !valid) {
            return Error::propagate(std::move(valid), "validate SF builder cache");
        }
        if (auto linkable = store::snapshot::check_link_filesystem(*cache_path, output_path); !linkable) {
            return linkable;
        }
        auto opened = mesh::storage::open_index(*cache_path / octree::storage::index_file_name);
        if (!opened) {
            return Error::propagate(std::move(opened), "open SF builder cache");
        }
        cache.emplace(std::move(*opened));
    }

    mesh::storage::OpenOptions create_options;
    create_options.preferred_extension = record.format;
    auto storage_result = mesh::storage::create(output_path, std::move(create_options));
    if (!storage_result) {
        return Error::propagate(std::move(storage_result), "create output terrain dataset \"" + output_path.string() + "\"");
    }
    if (auto written = store::inputs::write<inputs::Schema>(storage_result->base_path(), record); !written) {
        return written;
    }
    store::ThreadSafeStorage<mesh::storage::IndexedStorage> storage(std::move(storage_result.value()));

    auto dataset_srs_result = dataset.srs();
    if (!dataset_srs_result) {
        return Error::propagate(std::move(dataset_srs_result), "read dataset SRS");
    }
    const auto dataset_srs = *dataset_srs_result;
    auto dataset_bounds_result = dataset.bounds3d(false);
    if (!dataset_bounds_result) {
        return Error::propagate(std::move(dataset_bounds_result), "read dataset bounds");
    }
    const auto dataset_bounds = *dataset_bounds_result;
    auto dataset_coverage_result = dataset.geographic_coverage();
    if (!dataset_coverage_result) {
        return Error::propagate(std::move(dataset_coverage_result), "read dataset coverage");
    }
    const auto dataset_coverage = *dataset_coverage_result;

    const auto ecef_srs = gis::srs::ecef();
    auto ecef_bounds_result = gis::srs::ecef_coverage(dataset_srs, dataset_bounds);
    if (!ecef_bounds_result) {
        return Error::propagate(std::move(ecef_bounds_result), "transform dataset bounds to ECEF");
    }
    const auto ecef_bounds = *ecef_bounds_result;
    const auto space = octree::Space::earth();
    const auto root_node = expect(
        space.find_smallest_node_encompassing_bounds(ecef_bounds),
        "Dataset is outside the octree root node.");

    // Nodes within this radius contain no terrain: it is 100 km below the earth's mean radius.
    constexpr double terrain_free_radius = 6371008 - 100000;
    const auto within_terrain_free_radius = [](const radix::geometry::Aabb3d& bounds) {
        const glm::dvec3 farthest_corner = glm::max(glm::abs(bounds.min), glm::abs(bounds.max));
        return glm::length(farthest_corner) < terrain_free_radius;
    };
    // Below min_target_level, nodes touch the centre and are traversed without coverage checks.
    // From there on, those nodes are within the terrain free radius, which keeps the remaining
    // ones far enough from the centre for ecef2srs_coverage.
    ASSERT(glm::length(space.get_node_size_at_level(min_target_level)) < terrain_free_radius);
    const auto intersects_dataset = [&](const std::vector<radix::geometry::Aabb3d>& node_coverage) {
        for (const radix::geometry::Aabb3d& node_part : node_coverage) {
            if (node_part.max.z < dataset_bounds.min.z || dataset_bounds.max.z < node_part.min.z) {
                continue;
            }
            for (const radix::tile::SrsBounds& dataset_part : dataset_coverage) {
                if (radix::geometry::intersect(radix::tile::SrsBounds(node_part), dataset_part)) {
                    return true;
                }
            }
        }
        return false;
    };

    tbb::concurrent_vector<octree::Id> concurrent_targets;

    tbb::task_group tg;
    std::function<void(octree::Id)> process_node;

    // SRS objects are not shared between threads.
    tbb::enumerable_thread_specific<OGRSpatialReference> local_wgs84_srs([]() { return gis::srs::wgs84(); });
    tbb::enumerable_thread_specific<OGRSpatialReference> local_dataset_srs([&]() { return dataset_srs; });
    const auto native_bounds = radix::tile::SrsBounds(dataset_bounds);

    process_node = [&](octree::Id node) {
        // Check if node intersects with ecef bounds of dataset
        const auto node_bounds = space.get_node_bounds(node);
        if (!radix::geometry::intersect(node_bounds, ecef_bounds)) {
            return;
        }
        if (within_terrain_free_radius(node_bounds)) {
            return;
        }

        // Check if node coverage intersects with dataset in geographic coordinates
        if (node.level() >= min_target_level) {
            const auto node_coverage = gis::srs::ecef2srs_coverage(node_bounds, local_wgs84_srs.local());
            if (!node_coverage) {
                // Keep the node, dropping it could lose terrain.
                LOG_WARN("Failed to compute coverage of node {}: {}", node, node_coverage.error().to_string());
            } else if (!intersects_dataset(*node_coverage)) {
                return;
            }
        }

        if (node.level() < target_level) {
            if (auto children = node.children(); children.has_value()) {
                for (const auto &child : *children) {
                    tg.run([&, child] { process_node(child); });
                }
            }
        } else if (node.level() == target_level) {
            // Geographic coverage includes nodes outside of the native bounds of projected datasets.
            const auto windows = native_read_windows(local_dataset_srs.local(), native_bounds, node_bounds);
            if (windows && windows->empty()) {
                return;
            }
            concurrent_targets.push_back(node);
        }
    };

    tg.run([&] { process_node(root_node); });
    tg.wait(); // Wait for all tasks

    std::vector<octree::Id> target_nodes;
    target_nodes.assign(concurrent_targets.begin(), concurrent_targets.end());

    cli_support::ProgressIndicator progress(target_nodes.size());
    std::jthread progress_thread = progress.start_monitoring();

    // Clone dataset and SRSs for each thread
    tbb::enumerable_thread_specific<gis::Dataset> local_dataset([&]() {
        return dataset.clone();
    });
    tbb::enumerable_thread_specific<std::unique_ptr<OGRSpatialReference>> local_ecef_srs([&]() {
        return gis::srs::clone(ecef_srs);
    });
    tbb::enumerable_thread_specific<std::unique_ptr<OGRSpatialReference>> local_texture_srs([&]() {
        return gis::srs::clone(texture_srs);
    });
    tbb::enumerable_thread_specific<std::unique_ptr<OGRSpatialReference>> local_mesh_srs([&]() {
        return gis::srs::clone(mesh_srs);
    });

    // Decrease log level to error to avoid excessive logging during mesh building
    auto logger = cli_support::Log::get_logger();
    const auto original_level = logger->level();
    const auto new_level = spdlog::level::err;
    // Only set to if it's more restrictive than current level
    if (new_level >= original_level) {
        logger->set_level(new_level);
    }

    std::atomic_bool stopped = false;
    std::atomic<std::chrono::steady_clock::rep> last_checkpoint = std::chrono::steady_clock::now().time_since_epoch().count();
    // One task at a time saves the index once the checkpoint interval has passed.
    const auto checkpoint = [&]() -> Expected<void> {
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        auto last = last_checkpoint.load();
        if (now - std::chrono::steady_clock::duration(last) < store::snapshot::checkpoint_interval
            || !last_checkpoint.compare_exchange_strong(last, now.count())) {
            return {};
        }
        return storage.save_or_create_index();
    };
    tbb::task_group_context context;
    tbb::parallel_for(size_t(0), target_nodes.size(), [&](size_t i) {
        const auto &node = target_nodes[i];
        if (stop_requested && stop_requested()) {
            stopped = true;
            progress.task_finished();
            context.cancel_group_execution();
            return;
        }
        if (cache.has_value()) {
            const auto cached = cache->has(node);
            if (!cached) {
                LOG_ERROR("Failed to inspect cached node {}: {}", node, cached.error().to_string());
                progress.task_finished();
                context.cancel_group_execution();
                return;
            }
            if (cached.value()) {
                const auto linked = storage.link_from(node, *cache);
                if (!linked) {
                    LOG_ERROR("Failed to link cached node {}: {}", node, linked.error().to_string());
                    progress.task_finished();
                    context.cancel_group_execution();
                    return;
                }
                progress.task_finished();
                return;
            }
        }

        auto &dataset = local_dataset.local();
        auto &ecef_srs = *local_ecef_srs.local();
        auto &texture_srs = *local_texture_srs.local();
        auto &mesh_srs = *local_mesh_srs.local();

        const auto node_bounds = space.get_node_bounds(node);
        auto mesh_result = terrainbuilder::build_patch(
            dataset,
            ecef_srs,
            node_bounds,
            texture_srs,
            tile_provider,
            mesh_srs);

        if (mesh_result.has_value()) {
            const auto mesh = std::move(mesh_result.value());
            mesh::validate(mesh);
            const auto save_result = storage.save(node, mesh);
            if (!save_result) {
                LOG_ERROR(
                    "Failed to save mesh for node {}: {}",
                    node,
                    save_result.error().to_string());
                progress.task_finished();
                context.cancel_group_execution();
                return;
            }
        }

        if (const auto saved = checkpoint(); !saved) {
            LOG_ERROR("Failed to checkpoint the index: {}", saved.error().to_string());
            progress.task_finished();
            context.cancel_group_execution();
            return;
        }
        progress.task_finished();
    }, context);

    // Restore original level
    logger->set_level(original_level);

    if (context.is_group_execution_cancelled()) {
        progress_thread.request_stop();
    }
    progress_thread.join();

    auto finalized_storage = std::move(storage).release();
    if (context.is_group_execution_cancelled()) {
        if (auto saved = finalized_storage.save_index(); !saved) {
            return Error::propagate(std::move(saved), "checkpoint incomplete terrain storage");
        }
        if (stopped) {
            return Error::fail(Error::Code::Cancelled, "SF build cancelled; incomplete snapshot retained at " + finalized_storage.base_path().string());
        }
        return Error::fail(
            Error::Code::Internal, "failed to build all terrain patches; incomplete snapshot retained at " + finalized_storage.base_path().string());
    }

    auto finalized = sf::finalize_storage(finalized_storage);
    if (!finalized) {
        return Error::propagate(std::move(finalized), "finalize generated terrain storage \"" + finalized_storage.base_path().string() + "\"");
    }
    auto published = mesh::storage::publish(std::move(finalized_storage));
    if (!published) {
        return Error::propagate(std::move(published), "publish generated terrain storage \"" + output_path.string() + "\"");
    }
    return {};
}
}
