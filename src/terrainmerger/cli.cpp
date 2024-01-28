#include "cli.h"

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

cli::Args cli::parse(int argc, char **argv) {
    CLI::App app{"Terrain Merger"};
    // app.allow_windows_style_options();
    argv = app.ensure_utf8(argv);
    
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

    float simplification_factor = 0.25f;
    app.add_option("--simplify-factor", simplification_factor, "Mesh index simplification factor")
        ->check(CLI::Range(0.0f, 1.0f))
        ->excludes("--no-simplify");

    float simplification_target_error = 0.01f;
    app.add_option("--simplify-error", simplification_target_error, "Mesh simplification target error")
        ->check(CLI::Range(0.0f, 1.0f))
        ->excludes("--no-simplify");

    spdlog::level::level_enum log_level = spdlog::level::level_enum::info;
    app.add_option("--verbosity", log_level, "Verbosity level of logging")
        ->check(CLI::Range(0.0f, 1.0f));

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &e) {
        exit(app.exit(e));
    }

    Args args;
    args.input_paths = input_paths;
    args.output_path = output_path;
    args.log_level = log_level;
    if (!no_mesh_simplification) {
        SimplificationArgs simplification_args;
        simplification_args.simplification_factor = simplification_factor;
        args.simplification = simplification_args;
    }

    return args;
}
