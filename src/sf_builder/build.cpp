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

#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <ranges>
#include <span>
#include <stop_token>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fmt/core.h>
#include <glm/glm.hpp>
#include <glm/gtx/component_wise.hpp>
#include <libassert/assert.hpp>
#include <radix/geometry.h>
#include <spdlog/spdlog.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>

#include "build.h"
#include "cli_support/ProgressIndicator.h"
#include "cli_support/log.h"
#include "common_error_codes.h"
#include "earth.h"
#include "error/try.h"
#include "error/utils.h"
#include "geometry/geometry.h"
#include "gis/Dataset.h"
#include "gis/srs/System.h"
#include "gis/srs/ThreadLocalSystem.h"
#include "gis/srs/coverage.h"
#include "gis/srs/well_known.h"
#include "inputs.h"
#include "io/json/Value.h"
#include "LogScope.h"
#include "mem.h"
#include "mesh/SimpleMesh.h"
#include "mesh/io.h"
#include "mesh/storage.h"
#include "mesh/validate/debug_assert.h"
#include "mesh_builder.h"
#include "native_regions.h"
#include "octree/Id.h"
#include "octree/Space.h"
#include "octree/storage/index_format.h"
#include "parallel.h"
#include "sf/publish.h"
#include "stop.h"
#include "store/Checkpoint.h"
#include "store/Checkpointer.h"
#include "store/inputs.h"
#include "TileProvider.h"
#include "type_utils.h"

namespace sf {

namespace {
alp::Result<void, gis::MissingSrs, gis::Dataset::ComputeBoundsError, gis::srs::coverage::Error> log_dataset_overview(const gis::Dataset &dataset) {
    const std::string name = dataset.name();

    const uint32_t widthPx = dataset.raster_size().x;
    const uint32_t heightPx = dataset.raster_size().y;
    const uint32_t bands = dataset.band_count();

    ALP_TRY(const gis::srs::System datasetSrs, dataset.srs());
    ALP_TRY(const radix::tile::SrsAndHeightBounds bounds, dataset.bounds3d(true));
    ALP_TRY(const radix::geometry::Aabb3d boundsEcef, gis::srs::coverage::compute_ecef(datasetSrs, bounds));

    const octree::Space earth = octree::Space::earth();
    const auto enclosingNode =
        *ASSERT_VAL(earth.find_smallest_node_encompassing_bounds(boundsEcef));

    const int32_t scaleLevel = static_cast<int32_t>(std::floor(std::log2(
        earth.bounds().size().x / glm::compMax(boundsEcef.size()))));

    LOG_INFO("Dataset");
    LOG_INFO("  name        : {}", name);
    LOG_INFO("  size        : {} x {} px | bands={}", widthPx, heightPx, bands);
    LOG_INFO("  srs         : {}", datasetSrs);
    LOG_INFO("  bounds      : [{:.3f}, {:.3f}, {:.3f}] - [{:.3f}, {:.3f}, {:.3f}]",
             bounds.min.x, bounds.min.y, bounds.min.z,
             bounds.max.x, bounds.max.y, bounds.max.z);
    LOG_INFO("  bounds ecef : [{:.3f}, {:.3f}, {:.3f}] - [{:.3f}, {:.3f}, {:.3f}]",
             boundsEcef.min.x, boundsEcef.min.y, boundsEcef.min.z,
             boundsEcef.max.x, boundsEcef.max.y, boundsEcef.max.z);
    LOG_INFO("  octree      : enclosing node level={} coords=({}, {}, {})",
             enclosingNode.level(),
             enclosingNode.coords().x,
             enclosingNode.coords().y,
             enclosingNode.coords().z);
    LOG_INFO("  octree      : scale level={}", scaleLevel);
    return {};
}

std::unique_ptr<TileProvider> make_tile_provider(
    const std::optional<std::filesystem::path> &texture_base_path,
    const std::optional<uint32_t> min_texture_level,
    const std::optional<uint32_t> max_texture_level) {
    if (!texture_base_path.has_value()) {
        return nullptr;
    }

    GoogleMapboxTilePathProvider path_provider(texture_base_path.value());
    if (min_texture_level.has_value() || max_texture_level.has_value()) {
        return std::make_unique<ZoomRangeTileProvider<GoogleMapboxTilePathProvider>>(
            std::move(path_provider),
            min_texture_level,
            max_texture_level);
    } else {
        return std::make_unique<GoogleMapboxTilePathProvider>(std::move(path_provider));
    }
}

// The EPSG code of `srs`, or null if it has none.
io::json::Value epsg_code_or_null(const gis::srs::System &srs) {
    const std::optional<uint32_t> code_opt = srs.epsg_code();
    if (code_opt.has_value()) {
        return code_opt.value();
    } else {
        return nullptr;
    }
}
}

alp::Result<void> build_and_save_patch(const inputs::Patch &inputs, const std::filesystem::path &output_path) {
    const auto context = alp::make_context("building the patch for {} in {} from {} into {}", inputs.target_bounds, inputs.target_srs, inputs.dataset, output_path);

    ALP_TRY_CONTEXT(gis::Dataset dataset, gis::Dataset::open_raster(inputs.dataset), context);
    ALP_TRY_CONTEXT(log_dataset_overview(dataset), context);
    ALP_TRY_CONTEXT(const gis::srs::System mesh_srs, gis::srs::from_user_input(inputs.mesh_srs), context);
    ALP_TRY_CONTEXT(const gis::srs::System target_bounds_srs, gis::srs::from_user_input(inputs.target_srs), context);
    const std::unique_ptr<TileProvider> tile_provider = make_tile_provider(inputs.textures, inputs.min_texture_level, inputs.max_texture_level);
    const radix::geometry::Aabb3d &target_bounds = inputs.target_bounds;

    auto mesh_result = build_patch(dataset, target_bounds_srs, target_bounds, tile_provider.get(), mesh_srs);
    if (!mesh_result.has_value()) {
        ALP_TRY_CONTEXT(const NoTerrain no_terrain, std::move(mesh_result).error().split<NoTerrain>(), context);
        LOG_WARN("{:#}", no_terrain);
        return {};
    }
    const SimpleMesh mesh = std::move(mesh_result).value();

    LOG_INFO("Writing mesh to output path {}", output_path);

    const std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    const gis::srs::System texture_srs = gis::srs::webmercator();
    io::json::Object metadata{
        {"mesh_srs", epsg_code_or_null(mesh_srs)},
        {"mesh_srs_name", fmt::to_string(mesh_srs)},
        {"bounds_srs", epsg_code_or_null(target_bounds_srs)},
        {"bounds_srs_name", fmt::to_string(target_bounds_srs)},
        {"texture_srs", epsg_code_or_null(texture_srs)},
        {"texture_srs_name", fmt::to_string(texture_srs)},
        {"bounds", io::json::Object{
            {"min", io::json::Object{{"x", target_bounds.min.x}, {"y", target_bounds.min.y}}},
            {"max", io::json::Object{{"x", target_bounds.max.x}, {"y", target_bounds.max.y}}},
        }},
    };
    // TODO: add the texture bounds to the metadata
    ALP_TRY_CONTEXT(mesh::io::save_to_path(mesh, output_path, mesh::io::SaveOptions{.metadata = metadata, .mode = io::WriteMode::Replace}), context);
    const double seconds = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
    LOG_DEBUG("Writing mesh took {:.2f}s", seconds);
    LOG_INFO("Done writing mesh to {}", output_path);
    return {};
}

namespace {
using BuildError = alp::Error<difference_t<BuildMeshError, NoTerrain>, TileError, mesh::storage::Draft::SaveError>;

// Builds the mesh for `node` and saves it to `output`, or saves nothing if the dataset holds no terrain there.
alp::Result<void, BuildError> build_and_save_node(
    gis::Dataset &dataset,
    const octree::Space &space,
    const octree::Id &node,
    const gis::srs::System &ecef_srs,
    const TileProvider *tile_provider,
    const gis::srs::System &mesh_srs,
    mesh::storage::Draft &output) {
    auto mesh_result = build_patch(dataset, ecef_srs, space.get_node_bounds(node), tile_provider, mesh_srs);
    if (!mesh_result.has_value()) {
        ALP_TRY_CONTEXT(std::move(mesh_result).error().split<NoTerrain>(), fmt::format("building node {}", node));
        return {};
    }
    const SimpleMesh mesh = std::move(mesh_result).value();
    mesh::debug_assert_valid(mesh);
    ALP_TRY(output.save(node, mesh));
    return {};
}

// Nodes within this radius contain no terrain: it is the smallest distance from the earth's centre to its surface.
constexpr double terrain_free_radius = earth::smallest_radius;

// Whether the bounds lie so close to the earth's centre that they hold no terrain.
bool lies_within_terrain_free_radius(const radix::geometry::Aabb3d &bounds) {
    return geometry::compute_farthest_distance_to_bounds(bounds, glm::dvec3(0)) < terrain_free_radius;
}

// Whether a node's geographic coverage meets the dataset's coverage within its height range.
bool intersects_dataset(
    const std::vector<radix::geometry::Aabb3d> &node_coverage,
    const radix::tile::SrsAndHeightBounds &dataset_bounds,
    const std::vector<radix::tile::SrsBounds> &dataset_coverage) {
    for (const radix::geometry::Aabb3d &region_bounds : node_coverage) {
        if (region_bounds.max.z < dataset_bounds.min.z || dataset_bounds.max.z < region_bounds.min.z) {
            continue;
        }
        const radix::tile::SrsBounds horizontal_bounds(region_bounds);
        for (const radix::tile::SrsBounds &dataset_region_bounds : dataset_coverage) {
            if (radix::geometry::intersect_inclusive(horizontal_bounds, dataset_region_bounds)) {
                return true;
            }
        }
    }
    return false;
}

// The nodes at `target_level` below `root` that may hold terrain of the dataset.
alp::Result<std::vector<octree::Id>, Stopped, gis::srs::coverage::Error, gis::srs::coverage::NotGeographic> select_target_nodes(
    const octree::Space &space,
    const octree::Id &root,
    const octree::Id::Level target_level,
    const gis::srs::ThreadLocalSystem &dataset_srs,
    const radix::tile::SrsAndHeightBounds &dataset_bounds,
    const std::vector<radix::tile::SrsBounds> &dataset_coverage,
    const radix::geometry::Aabb3d &ecef_bounds,
    const std::stop_token &stop) {
    // Coverage is computed only for nodes from min_target_level on that reach past terrain_free_radius.
    // A node smaller than the radius that reaches past it cannot also reach the centre, where compute_from_ecef fails.
    ASSERT(glm::length(space.get_node_size_at_level(min_target_level)) < terrain_free_radius);

    const gis::srs::ThreadLocalSystem wgs84_srs(gis::srs::wgs84());
    const radix::tile::SrsBounds native_bounds(dataset_bounds);
    tbb::concurrent_vector<octree::Id> concurrent_targets;
    ALP_TRY(parallel::try_for_each(std::views::single(root), [&](const octree::Id &node, parallel::Feeder<octree::Id> &feeder) -> alp::Result<void, Stopped, gis::srs::coverage::Error, gis::srs::coverage::NotGeographic> {
        // Check if node intersects with ecef bounds of dataset
        const auto node_bounds = space.get_node_bounds(node);
        const auto context = alp::make_context("checking whether node {} may hold terrain", node);
        if (!radix::geometry::intersect_inclusive(node_bounds, ecef_bounds)) {
            return {};
        }
        if (lies_within_terrain_free_radius(node_bounds)) {
            return {};
        }

        // Check if node coverage intersects with dataset in geographic coordinates
        // Coarser nodes can reach the centre, so they are traversed without coverage checks.
        if (node.level() >= min_target_level) {
            ALP_TRY_CONTEXT(const std::vector<radix::geometry::Aabb3d> node_coverage, gis::srs::coverage::compute_from_ecef(node_bounds, wgs84_srs.get()), context);
            if (!intersects_dataset(node_coverage, dataset_bounds, dataset_coverage)) {
                return {};
            }
        }

        if (node.level() < target_level) {
            if (auto children = node.children(); children.has_value()) {
                for (const auto &child : *children) {
                    feeder.add(child);
                }
            }
        } else if (node.level() == target_level) {
            // Geographic coverage includes nodes outside of the native bounds of projected datasets.
            ALP_TRY_CONTEXT(const bool intersects, intersects_native_bounds(dataset_srs.get(), native_bounds, node_bounds), context);
            if (!intersects) {
                return {};
            }
            concurrent_targets.push_back(node);
        }
        return {};
    }, stop));

    std::vector<octree::Id> target_nodes(concurrent_targets.begin(), concurrent_targets.end());
    return target_nodes;
}

// Builds every node of `nodes` into `output`, linking those the checkpoint has instead of building them.
alp::Result<void, Stopped, gis::CplError, mesh::storage::Draft::LinkError, BuildError> build_nodes(
    const gis::Dataset &dataset,
    const octree::Space &space,
    const std::span<const octree::Id> nodes,
    const TileProvider *tile_provider,
    const gis::srs::ThreadLocalSystem &mesh_srs,
    mesh::storage::Draft &output,
    const mesh::storage::Checkpoint &checkpoint,
    const std::stop_token &stop) {
    const gis::srs::ThreadLocalSystem ecef_srs(gis::srs::ecef());

    cli_support::ProgressIndicator progress(nodes.size());
    std::jthread progress_thread = progress.start_monitoring();

    // Decrease log level to error to avoid excessive logging during mesh building
    LogScope log_scope(spdlog::level::err);

    // GDAL datasets are not thread-safe, so each thread reads through its own handle, opened on its first node.
    tbb::enumerable_thread_specific<std::optional<gis::Dataset>> thread_datasets;
    store::Checkpointer checkpointer;
    auto build_result = parallel::try_for_each(nodes, [&](const octree::Id &node) -> alp::Result<void, Stopped, gis::CplError, mesh::storage::Draft::LinkError, BuildError> {
        if (checkpoint.has(node)) {
            ALP_TRY(output.link_from(node, checkpoint));
        } else {
            std::optional<gis::Dataset> &thread_dataset = thread_datasets.local();
            if (!thread_dataset.has_value()) {
                ALP_TRY(thread_dataset, dataset.clone());
            }
            ALP_TRY(build_and_save_node(thread_dataset.value(), space, node, ecef_srs.get(), tile_provider, mesh_srs.get(), output));
        }
        checkpointer.save_if_due_or_warn(output);
        progress.task_finished();
        return {};
    }, stop);

    // Restore original level
    drop(std::move(log_scope));

    progress_thread.request_stop();
    progress_thread.join();

    return build_result;
}
}

alp::Result<void, Unsupported> check_target_level(const uint32_t target_level) {
    if (target_level < min_target_level || target_level > octree::Id::max_level()) {
        return alp::make_error(Unsupported{}, fmt::format("the target level {} is not supported, only levels {} to {} are", target_level, min_target_level, octree::Id::max_level()));
    }
    return {};
}

alp::Result<void> build_all_patches(
    const gis::Dataset &dataset,
    const octree::Id::Level target_level,
    const TileProvider *tile_provider,
    const gis::srs::System &mesh_srs,
    mesh::storage::Draft &output,
    const mesh::storage::Checkpoint &checkpoint,
    const std::stop_token &stop
) {
    ALP_TRY(check_target_level(target_level));

    ALP_TRY(const gis::srs::System dataset_system, dataset.srs());
    ALP_TRY(const radix::tile::SrsAndHeightBounds dataset_bounds, dataset.bounds3d());
    ALP_TRY(const std::vector<radix::tile::SrsBounds> dataset_coverage, dataset.geographic_coverage());
    ALP_TRY(const radix::geometry::Aabb3d ecef_bounds, gis::srs::coverage::compute_ecef(dataset_system, dataset_bounds));

    const auto space = octree::Space::earth();
    ALP_TRY(const octree::Id root_node, alp::to_result(
        space.find_smallest_node_encompassing_bounds(ecef_bounds),
        "the dataset lies outside the octree root node")
    );

    const gis::srs::ThreadLocalSystem dataset_srs(dataset_system);
    ALP_TRY(const std::vector<octree::Id> target_nodes, select_target_nodes(space, root_node, target_level, dataset_srs, dataset_bounds, dataset_coverage, ecef_bounds, stop));

    const gis::srs::ThreadLocalSystem shared_mesh_srs(mesh_srs);
    ALP_TRY(build_nodes(dataset, space, target_nodes, tile_provider, shared_mesh_srs, output, checkpoint, stop));
    return {};
}

alp::Result<void> build_and_publish_all_patches(
    const inputs::Build &inputs,
    const std::filesystem::path &output_path,
    const std::optional<std::filesystem::path> &checkpoint_path,
    const std::stop_token &stop) {
    const auto context = alp::make_context("building the SF storage at {} from {}", output_path, inputs.dataset);

    ALP_TRY_CONTEXT(check_target_level(inputs.target_level), context);

    ALP_TRY_CONTEXT(const gis::Dataset dataset, gis::Dataset::open_raster(inputs.dataset), context);
    ALP_TRY_CONTEXT(log_dataset_overview(dataset), context);
    ALP_TRY_CONTEXT(const gis::srs::System mesh_srs, gis::srs::from_user_input(inputs.mesh_srs), context);
    const std::unique_ptr<TileProvider> tile_provider = make_tile_provider(inputs.textures, inputs.min_texture_level, inputs.max_texture_level);

    mesh::storage::Checkpoint checkpoint;
    if (checkpoint_path.has_value()) {
        auto open_result = store::open_for_reuse<sf::inputs::Schema>(checkpoint_path.value(), output_path, inputs, mesh::storage::open);
        ALP_TRY_CONTEXT(checkpoint, std::move(open_result), context);
    }

    ALP_TRY_CONTEXT(mesh::storage::Draft output, mesh::storage::create(output_path, octree::storage::default_layout(), inputs.format), context);
    ALP_TRY_CONTEXT(store::inputs::write<sf::inputs::Schema>(output.base_path(), inputs), context);

    // check_target_level has bounded the level by octree::Id::max_level().
    const auto target_level = static_cast<octree::Id::Level>(inputs.target_level);
    ALP_TRY_CONTEXT(build_all_patches(dataset, target_level, tile_provider.get(), mesh_srs, output, checkpoint, stop), context);

    ALP_TRY_CONTEXT(sf::publish(std::move(output)), context);
    return {};
}

}
