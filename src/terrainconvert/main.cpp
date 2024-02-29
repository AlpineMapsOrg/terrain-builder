#include <filesystem>

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include "mesh/terrain_mesh.h"
#include "mesh/io.h"
#include "log.h"

void run(const std::filesystem::path& input_path, const std::filesystem::path& output_path) {
    const tl::expected<TerrainMesh, io::LoadMeshError> load_result = io::load_mesh_from_path(input_path);
    if (!load_result.has_value()) {
        LOG_ERROR("Failed to load mesh: {}", load_result.error().description());
        return;
    }
    const TerrainMesh mesh = load_result.value();

    const tl::expected<void, io::SaveMeshError> save_result = io::save_mesh_to_path(output_path, mesh);
    if (!save_result.has_value()) {
        LOG_ERROR("Failed to save mesh: {}", save_result.error().description());
        return;
    }
}

int main(int argc, char **argv) {
    assert(argc >= 0);

    CLI::App app{"Terrain Convert"};
    
    std::filesystem::path input_path;
    app.add_option("--input", input_path, "Path of tile to be converted")
        ->required()
        ->check(CLI::ExistingFile);

    std::filesystem::path output_path;
    app.add_option("--output", output_path, "Path to output the converted tile to")
        ->required();

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

    Log::init(log_level);

    run(input_path, output_path);
}
