/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2025 Martin Braunsperger
 * Copyright (C) 2025 Adam Celarek-Litofcenko
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

#include <cstdlib>
#include <exception>
#include <filesystem>
#include <optional>

#include "build.h"
#include "cli.h"
#include "cli_support/stop_signal.h"
#include "error/Result.h"
#include "error/log.h"
#include "error/try.h"
#include "inputs.h"
#include "io/filesystem_utils.h"
#include "log.h"
#include "mesh/storage.h"
#include "octree/storage/index_format.h"
#include "stop.h"
#include "storage.h"
#include "store/inputs.h"
#include "store/open.h"

namespace {

[[nodiscard]]
alp::Result<void> run(const cli::Args &args) {
    ALP_TRY(const mesh::storage::Storage input_storage, mesh::storage::open(args.input_path));

    dag::BuildOptions options{
        .clusters_per_partition = args.clusters_per_partition,
        .target_ratio = args.target_ratio,
        .relative_target_error = args.target_error,
        .texture_options = {
            .atlas = {.padding = args.texture_gutter},
            .bake = {.reprojection = {.gutter = args.texture_gutter}},
            .sizing = args.sizing_options,
            .charting = args.charting,
            .allow_texture_reuse = args.allow_texture_reuse,
        },
        .root_node = args.root_node,
        .include_mode = args.include_mode,
        .parallelize = args.parallelize,
    };
    // If the user specified neither target, fall back to a default error.
    if (!args.target_ratio && !args.target_error) {
        options.relative_target_error = 0.001f;
    }

    ALP_TRY(const dag::inputs::Build inputs, dag::inputs::make_build(args.input_path, options, args.level_range));

    dag::storage::Checkpoint checkpoint;
    if (args.checkpoint_path.has_value()) {
        auto open_result = store::open_for_reuse<dag::inputs::Schema>(args.checkpoint_path.value(), args.output_path, inputs, dag::storage::open);
        ALP_TRY(checkpoint, std::move(open_result));
    }

    if (args.write_debug_meshes) {
        ALP_TRY(io::filesystem::require_absent(dag::make_debug_mesh_path(args.output_path)));
    }
    ALP_TRY(dag::storage::Draft output_storage, dag::storage::create(args.output_path, octree::storage::default_layout()));
    ALP_TRY(store::inputs::write<dag::inputs::Schema>(output_storage.base_path(), inputs));

    std::optional<mesh::storage::Draft> debug_output;
    if (args.write_debug_meshes) {
        ALP_TRY(debug_output, mesh::storage::create(dag::make_debug_mesh_path(args.output_path), output_storage.resolver().layout(), ".glb"));
    }

    mesh::storage::Draft *const debug_output_ptr = debug_output.has_value() ? &debug_output.value() : nullptr;
    {
        const auto stop_guard = cli_support::stop_signal::install();
        ALP_TRY(dag::build_levels(input_storage, output_storage, options, args.level_range, checkpoint, debug_output_ptr, cli_support::stop_signal::token()));
    }

    ALP_TRY(dag::storage::publish(std::move(output_storage)));
    if (debug_output.has_value()) {
        ALP_TRY(mesh::storage::publish(std::move(*debug_output)));
    }
    return {};
}

} // namespace

int main(int argc, char **argv) {
    const cli::Args args = cli::parse(argc, argv);

    logging::Log::init(args.log_level);

    try {
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
    } catch (const std::exception &e) {
        LOG_ERROR("{}", e.what());
        return EXIT_FAILURE;
    }
}
