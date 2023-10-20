#include <chrono>
#include <filesystem>
#include <numeric>
#include <vector>

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

std::string format_secs_since(const std::chrono::high_resolution_clock::time_point &start) {
    return std::to_string(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start).count());
}

// TODO: setup cli interface
int main() {
    // Read MGI Dataset
    const DatasetPtr dataset = Dataset::make_shared("/mnt/c/Users/Admin/Downloads/innenstadt_gs_1m_mgi.tif");
    
    OGRSpatialReference webmercator;
    webmercator.importFromEPSG(3857);
    webmercator.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    OGRSpatialReference ecef;
    ecef.importFromEPSG(4978);
    ecef.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    // https://mapsneu.wien.gv.at/basemap/bmaporthofoto30cm/normal/google3857/18/90897/142994.jpeg
    const ctb::Grid grid = ctb::GlobalMercator();
    // const tile::Id root_tile = {16, {35748, 22724}, tile::Scheme::SlippyMap};
    // const tile::Id root_tile = {17, {71497, 45448}, tile::Scheme::SlippyMap};
    // const tile::Id root_tile = tile::Id{18, {142994, 90897}, tile::Scheme::SlippyMap}.parent().parent().parent();
    const std::array<tile::Id, 4> target_tiles = tile::Id {17, {71497, 45448}, tile::Scheme::SlippyMap}.children();
    // const tile::Id root_tile = {19, {285989, 181795}, tile::Scheme::SlippyMap};.

    for (unsigned int i = 0; i < target_tiles.size(); i++) {
        const tile::Id &root_tile = target_tiles[i];
        std::cout << root_tile << std::endl;

        tile::SrsBounds tile_bounds = grid.srsBounds(root_tile, false);
        tile::SrsBounds texture_bounds = tile_bounds;

        std::chrono::high_resolution_clock::time_point start;
        start = std::chrono::high_resolution_clock::now();
        TerrainMesh mesh = build_reference_mesh_tile(
            *dataset.get(),
            ecef,
            grid.getSRS(), tile_bounds,
            grid.getSRS(), texture_bounds,
            Border(0, 1, 1, 0),
            true);
        fmt::print("mesh building took {}s\n", format_secs_since(start));

        start = std::chrono::high_resolution_clock::now();
        std::optional<FiImage> texture = assemble_texture_from_tiles(
            grid, webmercator, texture_bounds,
            [](tile::Id tile_id) { return fmt::format("/mnt/e/Code/TU/2023S/Project/tiles/{}/{}/{}.jpeg", tile_id.zoom_level, tile_id.coords.y, tile_id.coords.x); });
        mesh.texture = std::move(texture);
        fmt::print("texture stitching took {}s\n", format_secs_since(start));

        start = std::chrono::high_resolution_clock::now();
        const std::filesystem::path mesh_path = fmt::format("/mnt/e/Code/TU/2023S/Project/meshes/{}/{}-{}.gltf", root_tile.zoom_level, root_tile.coords.y, root_tile.coords.x);
        save_mesh_as_gltf(mesh, mesh_path);
        fmt::print("mesh writing took {}s\n", format_secs_since(start));
        fmt::print("\n");
    }

    return 0;
}
