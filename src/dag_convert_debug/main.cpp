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

#include <array>
#include <cstdlib>
#include <filesystem>
#include <string>

#include "cli.h"
#include "codec.h"
#include "dag_node.h"
#include "log.h"
#include "error/Result.h"
#include "error/log.h"
#include "error/try.h"
#include "io/UnknownExtension.h"
#include "io/filesystem.h"
#include "io/filesystem_utils.h"
#include "path_utils.h"
#include "mesh/io.h"
#include "mesh/storage.h"
#include "cli_support/ProgressIndicator.h"
#include "storage.h"
#include "store/open.h"
#include "utils.h"

namespace {

[[nodiscard]]
alp::Result<void> export_node(const cli::Args &args) {
    const std::string extension = args.input_path.extension().string();
    if (extension != dag::codec::clustering_extension) {
        return io::make_unsupported_extension_error(extension, std::array { dag::codec::clustering_extension });
    }
    const std::filesystem::path node_path = paths::without_extension(args.input_path);
    const dag::codec::ClusterBatch codec;
    ALP_TRY(const dag::ClusterBatch node, codec.read(node_path));

    const mesh::Simple mesh = clustering_to_mesh(node.clustering);

    ALP_TRY(io::filesystem::require_absent(args.output_path));
    ALP_TRY(mesh::io::save_to_path(mesh, args.output_path));
    return {};
}

[[nodiscard]]
alp::Result<void> export_storage(const cli::Args &args) {
    ALP_TRY(const dag::storage::Storage input_storage, dag::storage::open(args.input_path, store::Accept::PublishedOrDraft));
    ALP_TRY(mesh::storage::Draft output_storage, mesh::storage::create(args.output_path, input_storage.resolver().layout(), ".glb"));

    size_t exported_count = 0;

    cli_support::ProgressIndicator progress(input_storage.index().map().size());
    auto progress_thread = progress.start_monitoring();

    for (const auto &[id, status] : input_storage.index().map()) {
        if (status == hierarchy::NodeStatus::Virtual) {
            progress.task_finished();
            continue;
        }

        const auto load_result = input_storage.load(id);
        if (!load_result.has_value()) {
            LOG_REPORT(load_result.error(), "skipping the node");
            progress.task_finished();
            continue;
        }

        const mesh::Simple mesh = clustering_to_mesh(load_result.value().clustering);

        const auto save_result = output_storage.save(id, mesh);
        if (!save_result.has_value()) {
            LOG_REPORT(save_result.error(), "skipping the node");
            progress.task_finished();
            continue;
        }

        exported_count++;
        progress.task_finished();
    }

    progress_thread.join();

    ALP_TRY(mesh::storage::publish(std::move(output_storage)));

    LOG_INFO("Exported {} debug meshes to {}", exported_count, args.output_path);
    return {};
}

[[nodiscard]]
alp::Result<void> run(const cli::Args &args) {
    ALP_TRY(const bool is_directory, io::filesystem::is_directory(args.input_path));
    if (is_directory) {
        return export_storage(args);
    } else {
        return export_node(args);
    }
}

} // namespace

int main(int argc, char **argv) {
    const cli::Args args = cli::parse(argc, argv);
    logging::Log::init(args.log_level);

    const alp::Result<void> result = run(args);
    if (!result.has_value()) {
        LOG_REPORT(result.error());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
