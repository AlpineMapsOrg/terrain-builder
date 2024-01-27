#ifndef CLI_H
#define CLI_H

#include <filesystem>
#include <vector>
#include <optional>

namespace cli {
    struct SimplificationArgs {
        double simplification_factor;
        // double simplification_target_error;
    };

    struct Args {
        std::filesystem::path output_path;
        std::vector<std::filesystem::path> input_paths;
        std::optional<SimplificationArgs> simplification;
    };

    Args parse(int argc, char **argv);
}; // namespace cli

#endif
