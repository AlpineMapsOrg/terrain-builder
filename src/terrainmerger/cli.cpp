#include "cli.h"

#include <span>
#include <algorithm>
#include <string>
#include <string_view>

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

using namespace cli;

Args cli::parse(int argc, const char * const * argv) {
    assert(argc >= 0);

    CLI::App app{"Terrain Merger"};
    // app.allow_windows_style_options();
    // argv = app.ensure_utf8(argv);
    
    std::vector<std::filesystem::path> input_paths;
    app.add_option("--input", input_paths, "Paths to tiles that should be merged")
        ->required()
        ->expected(-1)
        ->check(CLI::ExistingFile);

    std::filesystem::path output_path;
    app.add_option("--output", output_path, "Path to output the merged tile to")
        ->required();

    bool no_mesh_simplification = false;
    app.add_flag("--no-simplify", no_mesh_simplification, "Disable mesh simplification");

    std::optional<double> simplification_factor;
    app.add_option("--simplify-factor", *simplification_factor, "Mesh index simplification factor")
        ->check(CLI::Range(0.0f, 1.0f))
        ->excludes("--no-simplify");

    std::optional<double> simplification_target_error;
    app.add_option("--simplify-error", *simplification_target_error, "Mesh simplification target error")
        ->check(CLI::Range(0.0f, 1.0f))
        ->excludes("--no-simplify");

    spdlog::level::level_enum log_level = spdlog::level::level_enum::info;
    const std::map<std::string, spdlog::level::level_enum> log_level_names{
        {"off", spdlog::level::level_enum::off},
        {"critical", spdlog::level::level_enum::critical},
        {"error", spdlog::level::level_enum::err},
        {"warn", spdlog::level::level_enum::warn},
        {"info", spdlog::level::level_enum::info},
        {"debug", spdlog::level::level_enum::debug},
        {"trace", spdlog::level::level_enum::trace}};
    app.add_option("--verbosity", log_level, "Verbosity level of logging")
        ->transform(CLI::CheckedTransformer(log_level_names, CLI::ignore_case));

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &e) {
        exit(app.exit(e));
    }

    if (!simplification_factor.has_value() && !simplification_target_error.has_value()) {
        simplification_factor = 0.25;
    }

    Args args;
    args.input_paths = input_paths;
    args.output_path = output_path;
    args.log_level = log_level;
    if (!no_mesh_simplification) {
        SimplificationArgs simplification_args;
        simplification_args.factor = simplification_factor;
        simplification_args.max_error = simplification_target_error;
        args.simplification = simplification_args;
    }

    return args;
}
