#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <variant>

#include <spdlog/spdlog.h>

namespace cli {

// Which algorithm builds the tree over the frontier.
enum class Mode : uint8_t {
    PseudoLbvh,
    Lbvh,
    GatedPloc,
    Hploc,
};

// What every command takes.
struct BaseArgs {
    spdlog::level::level_enum log_level;
    size_t max_resident_bytes;
};

// What the build command takes.
struct BuildArgs : public BaseArgs {
    std::filesystem::path input_path;
    std::filesystem::path output_path;
    std::filesystem::path scratch_path;
    Mode mode;
    size_t max_batch_bytes;
};

using Args = std::variant<BuildArgs>;

Args parse(int argc, const char *const *argv);

}
