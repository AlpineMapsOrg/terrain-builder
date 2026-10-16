#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
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

// What the evaluate command takes.
struct EvaluateArgs : public BaseArgs {
    std::filesystem::path input_path;
    std::optional<std::filesystem::path> output_path;
    std::filesystem::path cameras_path;
    double threshold;
    bool no_cull = false;
    bool write_cut = false;
};

// What the check command takes.
struct CheckArgs : public BaseArgs {
    std::filesystem::path input_path;
};

using Args = std::variant<BuildArgs, EvaluateArgs, CheckArgs>;

Args parse(int argc, const char *const *argv);

}
