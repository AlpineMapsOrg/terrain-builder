#include "core/Application.h"
#include <CLI/App.hpp>
#include <CLI/Config.hpp>
#include <CLI/Formatter.hpp>
#include <log.h>

int main(int argc, char **argv)
{
#ifdef DEBUG
    Log::init(spdlog::level::trace);
#else
    Log::init(spdlog::level::trace);
#endif

    CLI::App cli_app{"Alpenite Browser"};
    cli_app.allow_windows_style_options();
    argv = cli_app.ensure_utf8(argv);

    std::vector<std::filesystem::path> octree_indices;
    cli_app.add_option("--octree-indices", octree_indices, "Path(s) to indexed octree folder(s).")
        ->check(CLI::ExistingDirectory);

    CLI11_PARSE(cli_app, argc, argv);

    Application app("Alpenite Browser", 1280, 720);

    app.run(octree_indices);

    return 0;
}