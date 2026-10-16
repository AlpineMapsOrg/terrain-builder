
#include <CLI/CLI.hpp>
#include <libassert/assert.hpp>
#include <spdlog/spdlog.h>

#include <cstdlib>
#include <filesystem>
#include <string>
#include <unordered_map>

#include "cli.h"
#include "bvh/Storage.h"
#include "storage/batch_limits.h"

using namespace cli;

namespace {

const std::unordered_map<std::string, Mode> MODE_NAMES{
    {"pseudo-lbvh", Mode::PseudoLbvh},
    {"lbvh", Mode::Lbvh},
    {"gated-ploc", Mode::GatedPloc},
    {"hploc", Mode::Hploc},
};

const std::unordered_map<std::string, spdlog::level::level_enum> LOG_LEVEL_NAMES{
    {"off", spdlog::level::level_enum::off},
    {"critical", spdlog::level::level_enum::critical},
    {"error", spdlog::level::level_enum::err},
    {"warn", spdlog::level::level_enum::warn},
    {"info", spdlog::level::level_enum::info},
    {"debug", spdlog::level::level_enum::debug},
    {"trace", spdlog::level::level_enum::trace},
};

// Adds the options every command takes to `command`.
void add_base_options(CLI::App &command, BaseArgs &args) {
    command.add_option("--max-resident-size", args.max_resident_bytes, "Target maximum memory to keep resident (e.g. for caches)")
        ->transform(CLI::AsSizeValue(false))
        ->default_val(DEFAULT_MAX_RESIDENT_BYTES)
        ->check(CLI::PositiveNumber);

    command.add_option("--verbosity", args.log_level, "Verbosity level of logging")
        ->transform(CLI::CheckedTransformer(LOG_LEVEL_NAMES, CLI::ignore_case))
        ->default_val("info");
}

// Adds the build command to `app`.
CLI::App &add_build_command(CLI::App &app, BuildArgs &args) {
    auto &command = *app.add_subcommand("build", "Build a bvh over a dag dataset");

    command.add_option("--input", args.input_path, "Path to the dag dataset to flatten")
        ->required()
        ->check(CLI::ExistingDirectory);

    command.add_option("--output", args.output_path, "Path to write the bvh run to")
        ->required();

    command.add_option("--scratch", args.scratch_path, "Path to keep the intermediate runs under")
        ->default_str("<output>-scratch");

    command.add_option("--mode", args.mode, "Which algorithm builds the tree over the frontier")
        ->transform(CLI::CheckedTransformer(MODE_NAMES, CLI::ignore_case))
        ->default_val("gated-ploc");

    command.add_option("--max-batch-size", args.max_batch_bytes, "Maximum size a batch file, e.g. 512MB or 1GB")
        ->transform(CLI::AsSizeValue(false))
        ->default_val(DEFAULT_MAX_BATCH_BYTES)
        ->check(CLI::PositiveNumber);

    add_base_options(command, args);
    return command;
}

// Adds the evaluate command to `app`.
CLI::App &add_evaluate_command(CLI::App &app, EvaluateArgs &args) {
    auto &command = *app.add_subcommand("evaluate", "Evaluate the cut a bvh selects for each camera");

    command.add_option("--input", args.input_path, "Path to the bvh run to evaluate")
        ->required()
        ->check(CLI::ExistingDirectory);

    command.add_option("--output", args.output_path, "Path to write the JSON result to instead of stdout");

    command.add_option("--cameras", args.cameras_path, "Path to a JSON file holding a camera set")
        ->required()
        ->check(CLI::ExistingFile);

    command.add_option("--threshold", args.threshold, "Largest error that may show, as a fraction of the screen height")
        ->default_val(2e-3)
        ->check(CLI::PositiveNumber);

    command.add_flag("--no-cull", args.no_cull, "Select clusters out of view too");

    command.add_flag("--write-cut", args.write_cut, "Write the selected clusters, not only their count");

    add_base_options(command, args);
    return command;
}

// Derives the default scratch path of a build writing to `output_path`.
std::filesystem::path derive_scratch_path(const std::filesystem::path &output_path) {
    // Kept beside the output, which is the one disk the caller has already chosen for this run.
    std::filesystem::path scratch_path = output_path;
    // A trailing separator leaves no filename to extend, and would name a directory inside the output.
    if (!scratch_path.has_filename()) {
        scratch_path = scratch_path.parent_path();
    }

    scratch_path += "-scratch";
    return scratch_path;
}

}

Args cli::parse(int argc, const char *const *argv) {
    DEBUG_ASSERT(argc >= 0);

    CLI::App app{"bvh_builder"};
    app.allow_windows_style_options();

    BuildArgs build_args;
    const CLI::App &build = add_build_command(app, build_args);

    EvaluateArgs evaluate_args;
    const CLI::App &evaluate = add_evaluate_command(app, evaluate_args);

    app.require_subcommand(1);

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &e) {
        std::exit(app.exit(e));
    }

    if (build) {
        if (build_args.scratch_path.empty()) {
            build_args.scratch_path = derive_scratch_path(build_args.output_path);
        }

        return build_args;
    }

    if (evaluate) {
        return evaluate_args;
    }

    UNREACHABLE("parsed without a command");
}
