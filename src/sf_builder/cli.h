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
    std::filesystem::path output_base_path;
    std::string output_format;
    uint32_t num_threads;
    bool overwrite_existing;
};

using Args = std::variant<SingleArgs, BatchArgs>;

Args parse(int argc, char **argv);

}
