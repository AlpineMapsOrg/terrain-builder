#include <chrono>
#include <filesystem>
#include <numeric>
#include <vector>

#include <CLI/CLI.hpp>
#include <FreeImage.h>
#include <fmt/core.h>
#include <gdal.h>
#include <gdal_priv.h>
#include <glm/glm.hpp>
#include <radix/geometry.h>

#include "Dataset.h"
#include "ctb/GlobalMercator.hpp"
#include "ctb/Grid.hpp"
#include "srs.h"

#include "fi_image.h"
#include "gltf_writer.h"
#include "terrain_mesh.h"
#include "mesh_builder.h"
#include "texture_assembler.h"

bool char_equals_ignore_case(const char a, const char b) {
    return std::tolower(std::char_traits<char>::to_int_type(a)) ==
           std::tolower(std::char_traits<char>::to_int_type(b));
}
bool string_equals_ignore_case(const std::string_view &a, const std::string_view &b) {
    return a.size() == b.size() &&
           std::equal(a.begin(), a.end(), b.begin(), char_equals_ignore_case);
}

std::string format_secs_since(const std::chrono::high_resolution_clock::time_point &start) {
    return std::to_string(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start).count());
}

int main(int argc, char** argv) {
    CLI::App app{"Terrain Builder"};
    app.allow_windows_style_options();
    argv = app.ensure_utf8(argv);

    std::string dataset_path;
    app.add_option("--dataset", dataset_path, "Path to a heightmap dataset file")
        ->required()
        ->check(CLI::ExistingFile);

    std::string texture_base_path;
    app.add_option("--textures", texture_base_path, "Path to a folder containing texture tiles in the format of {zoom}/{col}/{row}.jpeg")
        ->check(CLI::ExistingDirectory);

    unsigned int mesh_srs_code;
    app.add_option("--mesh-srs", mesh_srs_code, "EPSG code of the target srs of the mesh positions")
        ->default_val(4978);

    unsigned int target_srs_code;
    app.add_option("--target-srs", target_srs_code, "EPSG code of the srs of the target bounds or tile id")
        ->default_val(3857);

    auto *target = app.add_option_group("target");
    std::vector<double> target_bounds_data;
    target->add_option("--bounds", target_bounds_data, "Target bounds for the reference tile as \"{xmin} {ymin} {width} {height}\"")
        ->expected(4);
    std::vector<unsigned int> target_tile_data;
    target->add_option("--tile", target_tile_data, "Target tile id for the reference tile as \"{zoom} {x} {y}\"")
        ->expected(3)
        ->excludes("--bounds");
    target->require_option(1);

    tile::Scheme target_tile_scheme;
    std::map<std::string, tile::Scheme> scheme_str_map{{"slippymap", tile::Scheme::SlippyMap}, {"google", tile::Scheme::SlippyMap}, {"tms", tile::Scheme::Tms}};
    app.add_option("--scheme", target_tile_scheme, "Target tile id for the reference tile as \"{zoom} {x} {y}\"")
        ->default_val(tile::Scheme::SlippyMap)
        ->excludes("--bounds")
        ->transform(CLI::CheckedTransformer(scheme_str_map, CLI::ignore_case));

    std::string output_path;
    app.add_option("--output", output_path, "Path to which the reference mesh til will be written to (extension can be .gltf or .glb)");

    CLI11_PARSE(app, argc, argv);

    Dataset dataset(dataset_path);

    OGRSpatialReference tile_srs;
    tile_srs.importFromEPSG(target_srs_code);
    tile_srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    OGRSpatialReference mesh_srs;
    mesh_srs.importFromEPSG(mesh_srs_code);
    mesh_srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    OGRSpatialReference texture_srs;
    texture_srs.importFromEPSG(3857);
    texture_srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    const ctb::Grid grid = ctb::GlobalMercator();
    tile::SrsBounds tile_bounds;
    tile::SrsBounds texture_bounds;
    if (!target_bounds_data.empty()) {
        const glm::dvec2 tile_bounds_min(target_bounds_data[0], target_bounds_data[1]);
        const glm::dvec2 tile_bounds_size(target_bounds_data[2], target_bounds_data[3]);
        tile_bounds = tile::SrsBounds(tile_bounds_min, tile_bounds_min + tile_bounds_size);
    } else {
        const unsigned int zoom_level = target_tile_data[0];
        const glm::uvec2 tile_coords(target_tile_data[1], target_tile_data[2]);
        const tile::Id target_tile(zoom_level, tile_coords, target_tile_scheme);
        if (!tile_srs.IsSame(&grid.getSRS())) {
            throw std::runtime_error{"tile id is only allowed for webmercator srs"};
        }
        tile_bounds = grid.srsBounds(target_tile, false);
    }

    std::chrono::high_resolution_clock::time_point start;
    start = std::chrono::high_resolution_clock::now();
    TerrainMesh mesh = build_reference_mesh_tile(
        dataset,
        mesh_srs,
        tile_srs, tile_bounds,
        texture_srs, texture_bounds,
        Border(0, 1, 1, 0),
        true);
    fmt::print("mesh building took {}s\n", format_secs_since(start));

    start = std::chrono::high_resolution_clock::now();
    std::optional<FiImage> texture = assemble_texture_from_tiles(
        grid, texture_srs, texture_bounds,
        [=](tile::Id tile_id) { return fmt::format("{}/{}/{}/{}.jpeg", texture_base_path, tile_id.zoom_level, tile_id.coords.y, tile_id.coords.x); });
    mesh.texture = std::move(texture);
    fmt::print("texture stitching took {}s\n", format_secs_since(start));

    start = std::chrono::high_resolution_clock::now();
    // const std::filesystem::path mesh_path = fmt::format("/mnt/e/Code/TU/2023S/Project/meshes/{}/{}-{}.gltf", root_tile.zoom_level, root_tile.coords.y, root_tile.coords.x);
    save_mesh_as_gltf(mesh, output_path);
    fmt::print("mesh writing took {}s\n", format_secs_since(start));
    fmt::print("\n");

    return 0;
}
