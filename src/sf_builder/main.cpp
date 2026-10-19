#include <algorithm>
#include <execution>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <variant>
#include <vector>
#include <cstdint>
#include <cstdlib>

#include <fmt/core.h>
#include <glm/glm.hpp>
#include <glm/gtx/component_wise.hpp>
#include <radix/geometry.h>
#include <tbb/global_control.h>

#include "gdal/Dataset.h"
#include "cli.h"
#include "error/log.h"
#include "error/try.h"
#include "error/utils.h"
#include "log.h"
#include "srs/System.h"
#include "srs/transform_bounds.h"
#include "terrainbuilder.h"
#include "octree/Space.h"
#include "TileProvider.h"

#include "ctb/GlobalGeodetic.hpp"
#include "ctb/GlobalMercator.hpp"
#include "ctb/Grid.hpp"

radix::geometry::Aabb3d extend_bounds_to_3d(radix::geometry::Aabb2d bounds2d) {
    const double infinity = std::numeric_limits<double>::infinity();
    const glm::dvec3 min(bounds2d.min, -infinity);
    const glm::dvec3 max(bounds2d.max, infinity);
    return radix::geometry::Aabb3d(min, max);
}

alp::Result<radix::geometry::Aabb3d> parse_bounds_from_values(const std::vector<double> &data, const srs::System &srs) {
    const int32_t dim = srs.GetAxesCount();
    const size_t expected = (dim == 3) ? 6 : 4;

    if (data.size() != expected) {
        return alp::make_unexpected(fmt::format("Target bounds expects {} values for {}D SRS.", expected, dim));
    }

    if (data.size() == 4) {
        const glm::dvec2 min(data[0], data[2]);
        const glm::dvec2 max(data[0] + data[1], data[2] + data[3]);
        return extend_bounds_to_3d(radix::geometry::Aabb2d(min, max));
    } else if (data.size() == 6) {
        const glm::dvec3 min(data[0], data[2], data[4]);
        const glm::dvec3 max(data[0] + data[1], data[2] + data[3], data[4] + data[5]);
        return radix::geometry::Aabb3d(min, max);
    } else {
        UNREACHABLE();
    }
}

alp::Result<radix::geometry::Aabb3d> parse_bounds_from_tile(
    const std::vector<uint32_t> &data,
    const srs::System &srs) {

    // Determine the correct Grid type based on SRS
    std::optional<ctb::Grid> grid;
    const srs::System &webmercator_srs = srs::webmercator();
    const srs::System &wgs84_srs = srs::wgs84();
    if (srs.IsSame(&webmercator_srs)) {
        grid = ctb::GlobalMercator();
    } else if (srs.IsSame(&wgs84_srs)) {
        grid = ctb::GlobalGeodetic();
    } else {
        return alp::make_unexpected("Only WebMercator (EPSG:3857) or WGS84 (EPSG:4326) supported for --tile.");
    }

    const uint32_t zoom_level = data[0];
    const glm::uvec2 tile_coords(data[1], data[2]);
    const radix::tile::Id target_tile(zoom_level, tile_coords);

    return extend_bounds_to_3d(grid->srsBounds(target_tile, false));
}

alp::Result<radix::geometry::Aabb3d> parse_bounds_from_node(const std::vector<uint64_t> &data, const srs::System &srs) {
    // Validate node SRS (only ECEF)
    const srs::System &ecef_srs = srs::ecef();
    if (!srs.IsSame(&ecef_srs)) {
        return alp::make_unexpected("Only ECEF (EPSG:4978) supported for --node.");
    }

    const octree::Id::Level zoom_level = data[0];
    octree::Id target_node = octree::Id::root();

    if (data.size() == 2) {
        target_node = {zoom_level, data[1]};
    } else if (data.size() == 4) {
        const octree::Id::Coords coords(data[1], data[2], data[3]);
        target_node = {zoom_level, coords};
    } else {
        return alp::make_unexpected("Invalid number of args for --node. Expected 2 or 4.");
    }

    return octree::Space::earth().get_node_bounds(target_node);
}

alp::Result<radix::geometry::Aabb3d> parse_target_bounds(
    const std::vector<double> &bounds_data,
    const std::vector<uint64_t> &node_data,
    const std::vector<uint32_t> &tile_data,
    srs::System &srs) {

    if (!bounds_data.empty()) {
        return parse_bounds_from_values(bounds_data, srs);
    }

    if (!tile_data.empty()) {
        return parse_bounds_from_tile(tile_data, srs);
    }

    if (!node_data.empty()) {
        return parse_bounds_from_node(node_data, srs);
    }

    UNREACHABLE();
}

alp::Result<void, srs::TransformError> log_dataset_overview(const gdal::Dataset &dataset) {
    const std::string name = dataset.name();

    const uint32_t widthPx = dataset.widthInPixels();
    const uint32_t heightPx = dataset.heightInPixels();
    const uint32_t bands = dataset.n_bands();

    const auto datasetSrs = dataset.srs();
    const glm::dvec2 pixelSize = dataset.pixelSizeIn(datasetSrs);
    const auto bounds = dataset.bounds3d(true);

    ALP_TRY(const auto boundsEcef,
        srs::encompassing_bounds_transfer(datasetSrs, srs::ecef(), bounds));

    const octree::Space earth = octree::Space::earth();
    const auto enclosingNode =
        *ASSERT_VAL(earth.find_smallest_node_encompassing_bounds(boundsEcef));

    const int32_t scaleLevel = static_cast<int32_t>(std::floor(std::log2(
        earth.bounds().size().x / glm::compMax(boundsEcef.size()))));

    LOG_INFO("Dataset");
    LOG_INFO("  name        : {}", name);
    LOG_INFO("  size        : {} x {} px | bands={}", widthPx, heightPx, bands);
    LOG_INFO("  pixel size  : {:.3f} x {:.3f}", pixelSize.x, pixelSize.y);
    LOG_INFO("  srs         : {}", srs::friendly_name(datasetSrs));
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

std::unique_ptr<TileProvider> make_tile_provider(const cli::BaseArgs &args) {
    if (!args.texture_base_path.has_value()) {
        return nullptr;
    }

    GoogleMapboxTilePathProvider path_provider(args.texture_base_path.value());
    if (args.min_texture_level.has_value() || args.max_texture_level.has_value()) {
        return std::make_unique<ZoomRangeTileProvider<GoogleMapboxTilePathProvider>>(
            std::move(path_provider),
            args.min_texture_level,
            args.max_texture_level);
    } else {
        return std::make_unique<GoogleMapboxTilePathProvider>(std::move(path_provider));
    }
}

alp::Result<void> run(
    const cli::SingleArgs &args,
    gdal::Dataset &dataset,
    const srs::System &mesh_srs,
    const srs::System &texture_srs,
    const TileProvider *tile_provider) {
    ALP_TRY(srs::System target_srs, srs::from_user_input(args.target_srs_input));
    ALP_TRY(const radix::geometry::Aabb3d target_bounds, parse_target_bounds(
        args.target_bounds_data,
        args.target_node_data,
        args.target_tile_data,
        target_srs));

    ALP_TRY(terrainbuilder::build_and_save_patch(
        dataset,
        target_srs,
        target_bounds,
        texture_srs,
        tile_provider,
        mesh_srs,
        args.output_path));
    return {};
}

alp::Result<void> run(
    const cli::BatchArgs &args,
    gdal::Dataset &dataset,
    const srs::System &mesh_srs,
    const srs::System &texture_srs,
    const TileProvider *tile_provider) {
    std::optional<tbb::global_control> tbb_control;
    if (args.num_threads > 0) {
        LOG_DEBUG("Requesting max parallelism of {}", args.num_threads);
        tbb_control.emplace(tbb::global_control::max_allowed_parallelism, args.num_threads);
    }
    const auto actual_num_threads = tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism);
    const auto maybe_s = actual_num_threads == 1 ? "" : "s";
    LOG_INFO("Using {} thread{} for batch processing.", actual_num_threads, maybe_s);

    ALP_TRY(terrainbuilder::build_all_patches(
        dataset,
        args.target_level,
        texture_srs,
        tile_provider,
        mesh_srs,
        args.output_base_path,
        args.output_format,
        args.overwrite_existing));
    return {};
}

alp::Result<void> run(const cli::Args &args) {
    const cli::BaseArgs &base_args = std::visit([](const auto &args) -> const cli::BaseArgs & { return args; }, args);

    ALP_TRY(gdal::Dataset dataset, gdal::Dataset::open_raster(base_args.dataset_path));
    ALP_TRY(dataset.pixel_transform());
    ALP_TRY(log_dataset_overview(dataset));
    ALP_TRY(const auto mesh_srs, srs::from_user_input(base_args.mesh_srs_input));
    const auto texture_srs = srs::webmercator();
    const auto tile_provider = make_tile_provider(base_args);

    return std::visit([&](const auto &args) { return run(args, dataset, mesh_srs, texture_srs, tile_provider.get()); }, args);
}

int main(int argc, char **argv) {
    const cli::Args args = cli::parse(argc, argv);
    const auto log_level = std::visit([](const auto &args) { return args.log_level; }, args);
    Log::init(log_level);

    const alp::Result<void> result = run(args);
    if (!result.has_value()) {
        LOG_UNEXPECTED(result.error());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
