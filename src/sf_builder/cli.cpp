#include "cli.h"

#include <cstdlib>
#include <map>
#include <string>

#include <CLI/CLI.hpp>
#include <libassert/assert.hpp>
#include <spdlog/spdlog.h>

using namespace cli;

Args cli::parse(int argc, char **argv) {
    DEBUG_ASSERT(argc >= 0);

    CLI::App app{"sf_builder"};
    app.allow_windows_style_options();
    app.require_subcommand(1, 1);
    argv = app.ensure_utf8(argv);

    // === COMMON OPTIONS ===
    BaseArgs base_args;
    base_args.log_level = spdlog::level::level_enum::trace;

    const std::map<std::string, spdlog::level::level_enum> log_level_names{
        {"off", spdlog::level::level_enum::off},
        {"critical", spdlog::level::level_enum::critical},
        {"error", spdlog::level::level_enum::err},
        {"warn", spdlog::level::level_enum::warn},
        {"info", spdlog::level::level_enum::info},
        {"debug", spdlog::level::level_enum::debug},
        {"trace", spdlog::level::level_enum::trace}};

    app.add_option("--dataset", base_args.dataset_path, "Path to a heightmap dataset file")
        ->required()
        ->check(CLI::ExistingFile);
    app.add_option("--textures", base_args.texture_base_path, "Path to a folder containing texture tiles in the format of {zoom}/{col}/{row}.jpeg")
        ->check(CLI::ExistingDirectory);
    app.add_option("--mesh-srs", base_args.mesh_srs_input, "EPSG code of the target srs of the mesh positions")
        ->default_val("EPSG:4978");
    app.add_option("--min-texture-level", base_args.min_texture_level, "Minimum texture zoom level to use for assembling textures.")
        ->needs("--textures");
    app.add_option("--max-texture-level", base_args.max_texture_level, "Maximum texture zoom level to use for assembling textures.")
        ->needs("--textures");
    app.add_option("--verbosity", base_args.log_level, "Verbosity level of logging")
        ->transform(CLI::CheckedTransformer(log_level_names, CLI::ignore_case));

    // === SINGLE COMMAND ===
    auto *single = app.add_subcommand("single", "Build a single reference mesh")
                       ->fallthrough();
    SingleArgs single_args;

    auto *target = single->add_option_group("target");
    target->add_option("--bounds", single_args.target_bounds_data, "Target bounds for the reference mesh as \"{xmin} {width} {ymin} {height} [{zmin} {depth}]\"")
        ->expected(4, 6);
    target->add_option("--tile", single_args.target_tile_data, "Target tile id for the reference tile as \"{zoom} {x} {y}\"")
        ->expected(3);
    target->add_option("--node", single_args.target_node_data, "Target node id for the reference node as \"{zoom} {index}\" or \"{zoom} {x} {y} {z}\"")
        ->expected(2, 4);
    target->require_option(1);

    single->add_option("--output", single_args.output_path, "Output path were the mesh is written to (.terrain, .gltf or .glb)")
        ->required();

    single->add_option("--srs", single_args.target_srs_input, "EPSG code of the srs of the target bounds or id");
    single->callback([&]() {
        if (single_args.target_srs_input.empty()) {
            if (!single_args.target_bounds_data.empty()) {
                single_args.target_srs_input = "EPSG:4978";
            } else if (!single_args.target_tile_data.empty()) {
                single_args.target_srs_input = "EPSG:3857";
            } else if (!single_args.target_node_data.empty()) {
                single_args.target_srs_input = "EPSG:4978";
            }
        }
    });

    // === BATCH COMMAND ===
    auto *batch = app.add_subcommand("batch", "Build all nodes for a dataset at a given level")
                      ->fallthrough();
    BatchArgs batch_args;

    batch->add_option("--target-level", batch_args.target_level, "Level of detail for batch generation")
        ->required();
    batch->add_option("--output", batch_args.output_base_path, "Output path were the meshes are written to.")
        ->required();
    batch->add_option("--format", batch_args.output_format, "Output mesh format")
        ->check(CLI::IsMember({".glb", ".gltf", ".terrain"}))
        ->default_val(".terrain");
    batch_args.num_threads = 0;
    batch->add_option("--threads", batch_args.num_threads, "Number of threads to use")
        ->check(CLI::PositiveNumber);
    batch_args.overwrite_existing = false;
    batch->add_flag("--overwrite", batch_args.overwrite_existing, "Overwrite existing mesh files");

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &e) {
        exit(app.exit(e));
    }

    Args args;
    if (*single) {
        static_cast<BaseArgs &>(single_args) = base_args;
        args = std::move(single_args);
    } else if (*batch) {
        static_cast<BaseArgs &>(batch_args) = base_args;
        args = std::move(batch_args);
    }
    return args;
}
