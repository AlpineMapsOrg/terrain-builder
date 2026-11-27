/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2021 Adam Celarek-Litofcenko
 * Copyright (C) 2021 Martin Braunsperger
 * Copyright (C) 2026 Martin Braunsperger
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

#include <cstdint>
#include <cstdlib>
#include <limits>
#include <optional>
#include <variant>
#include <vector>

#include <fmt/core.h>
#include <glm/glm.hpp>
#include <libassert/assert.hpp>
#include <radix/geometry.h>
#include <tbb/global_control.h>

#include "build.h"
#include "cli.h"
#include "cli_support/log.h"
#include "cli_support/octree_id.h"
#include "cli_support/stop_signal.h"
#include "error/log.h"
#include "error/try.h"
#include "error/utils.h"
#include "gis/srs/System.h"
#include "gis/srs/well_known.h"
#include "gis/webmercator.h"
#include "inputs.h"
#include "octree/Space.h"
#include "stop.h"

radix::geometry::Aabb3d extend_bounds_to_3d(radix::geometry::Aabb2d bounds2d) {
    const double infinity = std::numeric_limits<double>::infinity();
    const glm::dvec3 min(bounds2d.min, -infinity);
    const glm::dvec3 max(bounds2d.max, infinity);
    return radix::geometry::Aabb3d(min, max);
}

alp::Result<radix::geometry::Aabb3d> parse_bounds_from_values(const std::vector<double> &data, const gis::srs::System &srs) {
    const size_t dim = srs.axes().size();
    const size_t expected = (dim == 3) ? 6 : 4;

    if (data.size() != expected) {
        return alp::make_error(fmt::format("Target bounds expects {} values for {}D SRS.", expected, dim));
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
    const gis::srs::System &srs) {

    if (!srs.is_same(gis::srs::webmercator())) {
        return alp::make_error("Only WebMercator (EPSG:3857) supported for --tile.");
    }

    const uint32_t zoom_level = data[0];
    const glm::uvec2 tile_coords(data[1], data[2]);
    const radix::tile::Id target_tile(zoom_level, tile_coords);

    return extend_bounds_to_3d(gis::webmercator::compute_tile_bounds(target_tile));
}

alp::Result<radix::geometry::Aabb3d> parse_bounds_from_node(const std::vector<uint64_t> &data, const gis::srs::System &srs) {
    // Validate node SRS (only ECEF)
    if (!srs.is_same(gis::srs::ecef())) {
        return alp::make_error("Only ECEF (EPSG:4978) supported for --node.");
    }

    ALP_TRY_CONTEXT(const octree::Id target_node, cli_support::parse_octree_id(data), "reading --node");

    return octree::Space::earth().get_node_bounds(target_node);
}

alp::Result<radix::geometry::Aabb3d> parse_target_bounds(
    const std::vector<double> &bounds_data,
    const std::vector<uint64_t> &node_data,
    const std::vector<uint32_t> &tile_data,
    gis::srs::System &srs) {

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

alp::Result<void> run(const cli::SingleArgs &args) {
    ALP_TRY(gis::srs::System target_srs, gis::srs::from_user_input(args.target_srs_input));
    ALP_TRY(const radix::geometry::Aabb3d target_bounds, parse_target_bounds(
        args.target_bounds_data,
        args.target_node_data,
        args.target_tile_data,
        target_srs));

    const sf::inputs::Patch inputs{
        .dataset = args.dataset_path,
        .textures = args.texture_base_path,
        .min_texture_level = args.min_texture_level,
        .max_texture_level = args.max_texture_level,
        .mesh_srs = args.mesh_srs_input,
        .target_srs = args.target_srs_input,
        .target_bounds = target_bounds,
    };
    ALP_TRY(sf::build_and_save_patch(inputs, args.output_path));
    return {};
}

alp::Result<void> run(const cli::BatchArgs &args) {
    std::optional<tbb::global_control> tbb_control;
    if (args.num_threads > 0) {
        LOG_DEBUG("Requesting max parallelism of {}", args.num_threads);
        tbb_control.emplace(tbb::global_control::max_allowed_parallelism, args.num_threads);
    }
    const auto actual_num_threads = tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism);
    const auto maybe_s = actual_num_threads == 1 ? "" : "s";
    LOG_INFO("Using {} thread{} for batch processing.", actual_num_threads, maybe_s);

    ALP_TRY(const sf::inputs::Build inputs, sf::inputs::make_build(args));
    const auto stop_guard = cli_support::stop_signal::install();
    ALP_TRY(sf::build_and_publish_all_patches(inputs, args.output_path, args.checkpoint_path, cli_support::stop_signal::token()));
    return {};
}

alp::Result<void> run(const cli::Args &args) {
    return std::visit([](const auto &args) { return run(args); }, args);
}

int main(int argc, char **argv) {
    const cli::Args args = cli::parse(argc, argv);
    const auto log_level = std::visit([](const auto &args) { return args.log_level; }, args);
    cli_support::Log::init(log_level);

    const alp::Result<void> result = run(args);
    if (!result.has_value()) {
        if (result.error().holds<Stopped>()) {
            LOG_INFO("{}", result.error());
            return cli_support::stop_signal::exit_code();
        }
        LOG_REPORT(result.error());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
