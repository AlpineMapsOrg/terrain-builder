/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2025 Martin Braunsperger
 * Copyright (C) 2025 Adam Celarek-Litofcenko
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
#include "cli_support/log.h"
#include "cli_support/stop_signal.h"
#include "inputs.h"
#include "mesh/storage.h"
#include "storage.h"
#include "store/inputs.h"
#include "store/snapshot.h"

int main(int argc, char **argv) {
    const cli::Args args = cli::parse(argc, argv);

    cli_support::Log::init(args.log_level);

    try {
        auto input_result = mesh::storage::open_folder_indexed(args.input_path);
        if (!input_result) {
            LOG_ERROR(
                "Failed to open input dataset {}: {}",
                args.input_path,
                input_result.error().to_string());
            return EXIT_FAILURE;
        }
        const mesh::storage::IndexedStorage input_storage = std::move(input_result.value());

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
            .write_debug_meshes = args.write_debug_meshes,
            .parallelize = args.parallelize,
        };
        // If the user specified neither target, fall back to a default error.
        if (!args.target_ratio && !args.target_error) {
            options.relative_target_error = 0.001f;
        }

        const auto record = dag::inputs::make_record(args.input_path, options, args.level_range);
        if (!record) {
            LOG_ERROR("Failed to record the build inputs: {}", record.error().to_string());
            return EXIT_FAILURE;
        }
        std::optional<dag::storage::IndexedStorage> cache;
        if (args.cache_path.has_value()) {
            const auto& cache_path = args.cache_path.value();
            if (const auto valid = store::inputs::validate_cache<dag::inputs::Schema>(cache_path, *record); !valid) {
                LOG_ERROR("Invalid cache {}: {}", cache_path, valid.error().to_string());
                return EXIT_FAILURE;
            }
            if (const auto linkable = store::snapshot::check_link_filesystem(cache_path, args.output_path); !linkable) {
                LOG_ERROR("Unusable cache {}: {}", cache_path, linkable.error().to_string());
                return EXIT_FAILURE;
            }
            auto cache_result = dag::storage::open_index(cache_path / octree::storage::index_file_name);
            if (!cache_result) {
                LOG_ERROR("Failed to open cache {}: {}", cache_path, cache_result.error().to_string());
                return EXIT_FAILURE;
            }
            cache.emplace(std::move(*cache_result));
        }
        if (args.write_debug_meshes) {
            if (const auto absent = store::snapshot::require_absent(dag::debug_mesh_path(args.output_path)); !absent) {
                LOG_ERROR("Debug mesh output must be new: {}", absent.error().to_string());
                return EXIT_FAILURE;
            }
        }

        auto output_result = dag::storage::create(args.output_path);
        if (!output_result) {
            LOG_ERROR("Failed to create output dataset {}: {}", args.output_path, output_result.error().to_string());
            return EXIT_FAILURE;
        }
        dag::storage::IndexedStorage output_storage = std::move(output_result.value());
        if (const auto written = store::inputs::write<dag::inputs::Schema>(output_storage.base_path(), *record); !written) {
            LOG_ERROR("Failed to write the input record: {}", written.error().to_string());
            return EXIT_FAILURE;
        }

        cli_support::stop_signal::install();
        const auto build_result = dag::build_levels(
            input_storage, output_storage, options, args.level_range, cache ? &cache.value() : nullptr, cli_support::stop_signal::requested);
        if (!build_result) {
            if (build_result.error().code() == Error::Code::Cancelled) {
                LOG_INFO("{}", build_result.error().to_string());
                return cli_support::stop_signal::exit_code();
            }
            LOG_ERROR("DAG build failed: {}", build_result.error().to_string());
            return EXIT_FAILURE;
        }
        const auto published = dag::storage::publish(std::move(output_storage));
        if (!published) {
            LOG_ERROR("Failed to publish output {}: {}", args.output_path, published.error().to_string());
            return EXIT_FAILURE;
        }

        return EXIT_SUCCESS;
    } catch (const std::exception &e) {
        LOG_ERROR("{}", e.what());
        return EXIT_FAILURE;
    }
}
