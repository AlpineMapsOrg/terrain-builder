/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2023 Martin Braunsperger
 * Copyright (C) 2023 Adrian Gawor
 * Copyright (C) 2023 Adam Celarek-Litofcenko
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
#include <filesystem>
#include <numeric>
#include <optional>
#include <string>
#include <variant>

#include "cli.h"
#include "cli_support/log.h"
#include "cli_support/stop_signal.h"
#include "cut.h"
#include "earth.h"
#include "error/Result.h"
#include "error/log.h"
#include "error/try.h"
#include "inputs.h"
#include "mask.h"
#include "merge.h"
#include "mesh/storage.h"
#include "sf/open.h"
#include "stop.h"

[[nodiscard]]
alp::Result<MeshMask, mask::LoadError> load_mask_from_path(const std::filesystem::path& path) {
    LOG_INFO("Loading mask file from {}", path);
    const glm::dvec2 radius_range = mask::pad_radius_range(earth::radius_range(), 2);
    ALP_TRY(MeshMask mask, mask::load_from_path(path, radius_range));
    LOG_DEBUG("Loaded mask successfully ({} vertices, {} triangles)", mask.vertex_count(), mask.face_count());
    return mask;
}

[[nodiscard]]
alp::Result<void> run(const cli::MergeArgs& args) {
    LOG_TRACE("Loading base dataset from {}", args.base_path);
    ALP_TRY(const mesh::storage::Storage base_dataset, sf::open(args.base_path));

    LOG_TRACE("Loading new dataset from {}", args.new_path);
    ALP_TRY(const mesh::storage::Storage new_dataset, sf::open(args.new_path));

    ALP_TRY(const sf_merger::inputs::Merge inputs, sf_merger::inputs::make_merge(args.base_path, args.new_path, args.mask_path));

    std::optional<MeshMask> mask;
    if (args.mask_path.has_value()) {
        ALP_TRY(mask, load_mask_from_path(*args.mask_path));
    }

    const auto stop_guard = cli_support::stop_signal::install();
    ALP_TRY(merge_datasets(base_dataset, new_dataset, args.output_path, mask, inputs, args.checkpoint_path, cli_support::stop_signal::token()));
    return {};
}

[[nodiscard]]
alp::Result<void> run(const cli::CutArgs& args) {
    LOG_TRACE("Loading input dataset from {}", args.input_path);
    ALP_TRY(const mesh::storage::Storage input_dataset, sf::open(args.input_path));
    ALP_TRY(const MeshMask mask, load_mask_from_path(args.mask_path));
    ALP_TRY(cut_dataset(input_dataset, mask, args.output_path, args.keep_inside));
    return {};
}

[[nodiscard]]
alp::Result<void> run(const cli::Args& args) {
    return std::visit([](const auto& args) { return run(args); }, args);
}

int main(int argc, char **argv) {
    const cli::Args args = cli::parse(argc, argv);
    const auto log_level = std::visit([](const auto& args) { return args.log_level; }, args);
    cli_support::Log::init(log_level);

    const std::string arg_str = std::accumulate(argv, argv + argc, std::string(),
                                                [](const std::string &acc, const char *arg) {
                                                    return acc + (acc.empty() ? "" : " ") + arg;
                                                });
    LOG_DEBUG("Running with: {}", arg_str);

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
