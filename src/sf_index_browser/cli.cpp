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

#include "cli.h"

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>
#include <libassert/assert.hpp>

#include "cli_support/log.h"

using namespace cli;

Args cli::parse(int argc, const char * const * argv) {
    DEBUG_ASSERT(argc >= 0);

    Args args;
    CLI::App app{"sf_index_browser"};
    app.positionals_at_end(false);
    app.allow_windows_style_options(false);

    app.add_option("dataset,--dataset", args.dataset_path, "Folder or index file of dataset to browser")
        ->check(CLI::ExistingPath)
        ->required();

    args.full_view = false;
    app.add_flag("--full", args.full_view, "Start at octree root instead of dataset root.");

    app.add_option("--verbosity", args.log_level, "Verbosity level of logging")
        ->transform(CLI::CheckedTransformer(cli_support::log_level_names(), CLI::ignore_case))
        ->default_val(spdlog::level::level_enum::info);

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &e) {
        exit(app.exit(e));
    }

    return args;
}
