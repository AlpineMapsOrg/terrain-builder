/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2024 Martin Braunsperger
 * Copyright (C) 2024 Adam Celarek-Litofcenko
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
#include <expected>
#include <filesystem>

#include <glm/glm.hpp>

#include "error/Result.h"
#include "error/log.h"
#include "error/try.h"
#include "mesh/SimpleMesh.h"
#include "mesh/io.h"
#include "opencv_utils.h"
#include "cli_support/log.h"
#include "cli.h"

[[nodiscard]]
alp::Result<void> run(const cli::Args& args) {
    LOG_INFO("Loading input mesh...");
    ALP_TRY(SimpleMesh mesh, mesh::io::load_from_path(args.input_path));

    if (args.texture_resolution.has_value()) {
        if (mesh.texture.has_value()) {
            cv::Mat& texture = mesh.texture.value();
            glm::uvec2 target_resolution = args.texture_resolution.value();
            LOG_INFO("Resizing mesh texture to {}x{}", target_resolution.x, target_resolution.y);
            rescale_texture_inplace(texture, target_resolution);
        } else {
            LOG_WARN("--texture-resolution specified but no texture present");
        }
    }

    LOG_INFO("Writing output mesh...");
    ALP_TRY(mesh::io::save_to_path(mesh, args.output_path, {.mode = io::WriteMode::Replace}));
    return {};
}

int main(int argc, char **argv) {
    const cli::Args args = cli::parse(argc, argv);
    cli_support::Log::init(args.log_level);

    const alp::Result<void> result = run(args);
    if (!result.has_value()) {
        LOG_REPORT(result.error());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
