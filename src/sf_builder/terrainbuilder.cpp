#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <optional>
#include <ranges>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fmt/core.h>
#include <glm/glm.hpp>
#include <opencv2/core/mat.hpp>
#include <radix/geometry.h>

#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>

#include "gdal/Dataset.h"
#include "ctb/GlobalMercator.hpp"
#include "ctb/Grid.hpp"
#include "srs/System.h"
#include "srs/SharedSystem.h"
#include "srs/transform.h"
#include "srs/transform_bounds.h"

#include "octree/Id.h"
#include "octree/Space.h"
#include "octree/Storage.h"
#include "octree/disk/layout/strategy/LevelAndCoordinateDirectories.h"
#include "octree/utils.h"

#include "ProgressIndicator.h"
#include "error/log.h"
#include "error/try.h"
#include "error/utils.h"
#include "LogScope.h"
#include "log.h"
#include "mem.h"
#include "mesh/SimpleMesh.h"
#include "mesh/io.h"
#include "mesh_builder.h"
#include "terrainbuilder.h"
#include "texture_assembler.h"
#include "TileProvider.h"
#include "mesh/validate.h"
#include "parallel.h"

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

alp::Result<std::optional<SimpleMesh>, BuildPatchError> build_patch(
    gdal::Dataset &dataset,
    const srs::System &target_bounds_srs,
    const radix::geometry::Aabb3d &target_bounds,
    const srs::System &texture_srs,
    const TileProvider* tile_provider,
    const srs::System &mesh_srs) {
    const ctb::Grid grid = ctb::GlobalMercator();
    radix::tile::SrsBounds texture_bounds;

    std::chrono::high_resolution_clock::time_point start;
    start = std::chrono::high_resolution_clock::now();
    LOG_INFO("Building mesh...");
    alp::Result<SimpleMesh, BuildMeshError> mesh_result = build_reference_mesh_patch(
        dataset,
        mesh_srs,
        target_bounds_srs, target_bounds,
        texture_srs, texture_bounds);
    if (!mesh_result.has_value()) {
        auto split_result = std::move(mesh_result).error().split<NoTerrain>();
        if (!split_result.has_value()) {
            return alp::propagate(std::move(split_result).error());
        }
        LOG_WARN("{:#}", split_result.value());
        return std::nullopt;
    }
    SimpleMesh mesh = mesh_result.value();
    LOG_DEBUG("Mesh building took {}s", format_secs_since(start));
    LOG_INFO("Finished building mesh geometry");

    if (tile_provider != nullptr) {
        start = std::chrono::high_resolution_clock::now();
        LOG_INFO("Assembling mesh texture");
        ALP_TRY(
            std::optional<AssembledTexture> texture,
            assemble_texture_from_tiles(
                grid,
                texture_srs,
                texture_bounds,
                *tile_provider));
        if (texture.has_value()) {
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

alp::Result<void, BuildPatchError, mesh::io::Error> build_and_save_patch(
    gdal::Dataset &dataset,
    const srs::System &target_bounds_srs,
    const radix::geometry::Aabb3d &target_bounds,
    const srs::System &texture_srs,
    const TileProvider *tile_provider,
    const srs::System &mesh_srs,
    const std::filesystem::path &output_path) {
    ALP_TRY(
        std::optional<SimpleMesh> mesh_opt,
        build_patch(
            dataset,
            target_bounds_srs,
            target_bounds,
            texture_srs,
            tile_provider,
            mesh_srs));
    if (!mesh_opt.has_value()) {
        return {};
    }
    const SimpleMesh mesh = std::move(mesh_opt).value();

    LOG_INFO("Writing mesh to output path {}", output_path);

    std::chrono::high_resolution_clock::time_point start;
    start = std::chrono::high_resolution_clock::now();
    // TODO: use a JSON libary instead
    std::unordered_map<std::string, std::string> metadata;
    metadata["mesh_srs"] = srs::friendly_name(mesh_srs);
    metadata["bounds_srs"] = srs::friendly_name(target_bounds_srs);
    metadata["texture_srs"] = srs::friendly_name(texture_srs);
    metadata["bounds"] = fmt::format(
        "{{ \"min\": {{ \"x\": {}, \"y\": {} }}, \"max\": {{ \"x\": {}, \"y\": {} }} }}",
        target_bounds.min.x, target_bounds.min.y, target_bounds.max.x, target_bounds.max.y);
    ALP_TRY(mesh::io::save_to_path(mesh, output_path, mesh::io::SaveOptions{.metadata = metadata}));
    LOG_DEBUG("Writing mesh took {}s", format_secs_since(start));
    LOG_INFO("Done writing mesh to {}", output_path);
    return {};
}

namespace {
// An error from building and saving a node.
using BuildError = alp::Error<BuildPatchError, octree::Storage::SaveError>;

// Builds the mesh for `node` and saves it to `storage`, or saves nothing if the dataset holds no terrain there.
alp::BasicResult<void, BuildError> build_and_save_node(
    gdal::Dataset &dataset,
    const octree::Space &space,
    const octree::Id &node,
    const srs::System &ecef_srs,
    const srs::System &texture_srs,
    const TileProvider *tile_provider,
    const srs::System &mesh_srs,
    octree::Storage &storage) {
    ALP_TRY_CONTEXT(
        std::optional<SimpleMesh> mesh_opt,
        build_patch(
            dataset,
            ecef_srs,
            space.get_node_bounds(node),
            texture_srs,
            tile_provider,
            mesh_srs),
        fmt::format("building node {}", node));
    if (!mesh_opt.has_value()) {
        return {};
    }

    const SimpleMesh mesh = std::move(mesh_opt).value();
    mesh::validate(mesh);
    ALP_TRY(storage.save(node, mesh));
    return {};
}
}

alp::Result<void> build_all_patches(
    gdal::Dataset &dataset,
    const octree::Id::Level target_level,
    const srs::System &texture_srs,
    const TileProvider *tile_provider,
    const srs::System &mesh_srs,
    const std::filesystem::path &output_base_path,
    const std::string &output_format,
    const bool overwrite_existing
) {
    ALP_TRY(octree::Storage storage, octree::open_folder(
        output_base_path,
        false,
        octree::OpenOptions {
            .preferred_extension_with_dot = output_format
        }
    ));
    storage.settings().allow_overwrite = overwrite_existing;

    const srs::SharedSystem dataset_srs(dataset.srs());
    const auto dataset_bounds = dataset.bounds3d(true);

    const srs::SharedSystem ecef_srs(srs::ecef());
    const srs::SharedSystem shared_texture_srs(texture_srs);
    const srs::SharedSystem shared_mesh_srs(mesh_srs);
    ALP_TRY(const auto ecef_bounds, srs::encompassing_bounds_transfer(dataset_srs.get(), ecef_srs.get(), dataset_bounds));
    const auto space = octree::Space::earth();
    ALP_TRY(const octree::Id root_node, alp::to_expected(
        space.find_smallest_node_encompassing_bounds(ecef_bounds),
        "the dataset lies outside the octree root node")
    );

    tbb::concurrent_vector<octree::Id> concurrent_targets;

    ALP_TRY(parallel::try_for_each(std::views::single(root_node), [&](const octree::Id &node, parallel::Feeder<octree::Id> &feeder) -> alp::Result<void, srs::TransformError> {
        // Check if node intersects with ecef bounds of dataset
        const auto node_bounds = space.get_node_bounds(node);
        if (!radix::geometry::intersect(node_bounds, ecef_bounds)) {
            return {};
        }

        // Check if node bounds in dataset srs intersect with dataset
        ALP_TRY(const srs::Transform transform_ecef_dataset, srs::get_transform(ecef_srs.get(), dataset_srs.get()));
        ALP_TRY(const auto node_bounds_dataset_srs, srs::encompassing_bounds_transfer(
            transform_ecef_dataset, node_bounds, 7, 3));
        if (!radix::geometry::intersect(node_bounds_dataset_srs, dataset_bounds)) {
            return {};
        }

        if (node.level() < target_level) {
            if (auto children = node.children(); children.has_value()) {
                for (const auto &child : *children) {
                    feeder.add(child);
                }
            }
        } else if (node.level() == target_level) {
            concurrent_targets.push_back(node);
        }
        return {};
    }));

    std::vector<octree::Id> target_nodes;
    target_nodes.assign(concurrent_targets.begin(), concurrent_targets.end());

    ProgressIndicator progress(target_nodes.size());
    std::jthread progress_thread = progress.start_monitoring();

    // Clone dataset for each thread
    tbb::enumerable_thread_specific<gdal::Dataset> local_dataset([&]() {
        return dataset.clone();
    });

    // Decrease log level to error to avoid excessive logging during mesh building
    LogScope log_scope(spdlog::level::err);

    alp::BasicResult<void, BuildError> build_result = parallel::try_for_each(target_nodes, [&](const octree::Id &node) -> alp::BasicResult<void, BuildError> {
        if (!overwrite_existing && storage.has(node)) {
            progress.task_finished(); // TODO: correctly handle virtual nodes
            return {};
        }

        auto &dataset = local_dataset.local();

        auto result = build_and_save_node(
            dataset,
            space,
            node,
            ecef_srs.get(),
            shared_texture_srs.get(),
            tile_provider,
            shared_mesh_srs.get(),
            storage);

        progress.task_finished();
        return result;
    });

    // Restore original level
    drop(std::move(log_scope));

    progress_thread.request_stop();
    progress_thread.join();

    ALP_TRY(std::move(build_result));
    ALP_TRY(storage.save_or_create_index());
    return {};
}
}
