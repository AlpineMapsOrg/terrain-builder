/*****************************************************************************
 * AlpineMaps.org
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

#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include <radix/geometry.h>

#include "cli.h"
#include "error/Result.h"
#include "error/try.h"
#include "io/FileError.h"
#include "io/envelope.h"
#include "io/filesystem.h"

namespace sf::inputs {

// Everything that determines the meshes of a batch build.
// A checkpoint is reused only by a build with equal inputs.
struct Build {
    // Optional members defeat zpp_bits' member counting.
    using serialize = zpp::bits::members<8>;

    std::string dataset;
    std::optional<std::string> textures;
    std::optional<std::uint32_t> min_texture_level;
    std::optional<std::uint32_t> max_texture_level;
    std::string mesh_srs;
    std::uint32_t target_level = 0;
    std::string format;
    // Bump on changes that alter the produced meshes.
    std::uint32_t processing_version = 1;

    bool operator==(const Build&) const = default;
};

using Schema = io::envelope::PayloadSchema<"sf_builder.Inputs", io::envelope::Version<1, Build>>;

// What a single build reads and builds.
struct Patch {
    std::filesystem::path dataset;
    std::optional<std::filesystem::path> textures;
    std::optional<std::uint32_t> min_texture_level;
    std::optional<std::uint32_t> max_texture_level;
    std::string mesh_srs;
    std::string target_srs;
    radix::geometry::Aabb3d target_bounds;
};

// The inputs that the meshes of a batch build with `args` depend on.
[[nodiscard]]
inline alp::Result<Build, io::FileError> make_build(const cli::BatchArgs& args) {
    ALP_TRY(const std::filesystem::path dataset_path, io::filesystem::absolute(args.dataset_path));
    Build build;
    build.dataset = dataset_path.lexically_normal().string();
    if (args.texture_base_path.has_value()) {
        ALP_TRY(const std::filesystem::path textures_path, io::filesystem::absolute(args.texture_base_path.value()));
        build.textures = textures_path.lexically_normal().string();
    }
    build.min_texture_level = args.min_texture_level;
    build.max_texture_level = args.max_texture_level;
    build.mesh_srs = args.mesh_srs_input;
    build.target_level = args.target_level;
    build.format = args.output_format;
    return build;
}

} // namespace sf::inputs
