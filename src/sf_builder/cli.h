/*****************************************************************************
 * AlpineMaps.org
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

#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include <radix/tile.h>
#include <spdlog/spdlog.h>

#include "octree/Id.h"

namespace cli {

struct BaseArgs {
    spdlog::level::level_enum log_level;
    std::filesystem::path dataset_path;
    std::optional<std::filesystem::path> texture_base_path;
    std::string mesh_srs_input;
    std::optional<uint32_t> min_texture_level;
    std::optional<uint32_t> max_texture_level;
};

struct SingleArgs : public BaseArgs {
    std::filesystem::path output_path;
    std::vector<double> target_bounds_data;
    std::vector<uint32_t> target_tile_data;
    std::vector<uint64_t> target_node_data;
    std::string target_srs_input;
};

struct BatchArgs : public BaseArgs {
    octree::Id::Level target_level;
    std::filesystem::path output_path;
    std::string output_format;
    uint32_t num_threads = 0;
    std::optional<std::filesystem::path> checkpoint_path;
};

using Args = std::variant<SingleArgs, BatchArgs>;

Args parse(int argc, char **argv);

}
