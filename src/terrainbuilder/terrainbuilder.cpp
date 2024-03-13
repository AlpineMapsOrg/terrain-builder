#include <chrono>
#include <vector>

#include <fmt/core.h>
#include <glm/glm.hpp>
#include <radix/geometry.h>

#include "Dataset.h"
#include "ctb/GlobalMercator.hpp"
#include "ctb/Grid.hpp"
#include "srs.h"

#include "log.h"
#include "mesh/io.h"
#include "mesh/terrain_mesh.h"
#include "mesh_builder.h"
#include "texture_assembler.h"
#include "tile_provider.h"
#include "terrainbuilder.h"

namespace terrainbuilder {

std::string format_secs_since(const std::chrono::high_resolution_clock::time_point &start) {
    const auto duration = std::chrono::high_resolution_clock::now() - start;
    const double seconds = std::chrono::duration<double>(duration).count();
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << seconds;
    return ss.str();
}

class BasemapSchemeTilePathProvider : public TilePathProvider {
public:
    BasemapSchemeTilePathProvider(std::filesystem::path base_path)
        : base_path(base_path) {}

    std::optional<std::filesystem::path> get_tile_path(const tile::Id tile_id) const override {
        return fmt::format("{}/{}/{}/{}.jpeg", this->base_path.generic_string(), tile_id.zoom_level, tile_id.coords.y, tile_id.coords.x);
    }

private:
    std::filesystem::path base_path;
};

void build(
    Dataset &dataset,
    const std::optional<std::filesystem::path> texture_base_path,
    const OGRSpatialReference &mesh_srs,
    const OGRSpatialReference &tile_srs,
    const OGRSpatialReference &texture_srs,
    const tile::SrsBounds &tile_bounds,
    const std::filesystem::path &output_path) {
    const ctb::Grid grid = ctb::GlobalMercator();

    tile::SrsBounds target_tile_bounds(tile_bounds);
    tile::SrsBounds texture_bounds;

    std::chrono::high_resolution_clock::time_point start;
    start = std::chrono::high_resolution_clock::now();
    LOG_INFO("Building mesh...");
    tl::expected<TerrainMesh, mesh::BuildError> mesh_result = mesh::build_reference_mesh_tile(
        dataset,
        mesh_srs,
        tile_srs, target_tile_bounds,
        texture_srs, texture_bounds,
        Border(0, 1, 1, 0),
        true);
    if (!mesh_result.has_value()) {
        const mesh::BuildError error = mesh_result.error();
        if (error == mesh::BuildError::OutOfBounds) {
            const tile::SrsBounds dataset_bounds = dataset.bounds();
            const tile::Id dataset_largest_tile = grid.findLargestContainedTile(dataset_bounds).value().to(tile::Scheme::SlippyMap);
            const tile::Id dataset_encompassing_tile = grid.findSmallestEncompassingTile(dataset_bounds).value().to(tile::Scheme::SlippyMap);
            const tile::Id target_largest_tile = grid.findLargestContainedTile(tile_bounds).value().to(tile::Scheme::SlippyMap);
            const tile::Id target_encompassing_tile = grid.findSmallestEncompassingTile(tile_bounds).value().to(tile::Scheme::SlippyMap);
            LOG_ERROR("Target bounds are fully outside of dataset region\n"
                      "Dataset {{\n"
                      "\tBounds {{x={}, y={}, w={}, h={}}}.\n"
                      "\tLargest Contained Tile {{zoom={}, x={}, y={}}}.\n"
                      "\tSmallest Encompassing Tile {{zoom={}, x={}, y={}}}.\n"
                      "}}\n"
                      "Target {{\n"
                      "\tBounds {{x={}, y={}, w={}, h={}}}.\n"
                      "\tLargest Contained Tile {{zoom={}, x={}, y={}}}.\n"
                      "\tSmallest Encompassing Tile {{zoom={}, x={}, y={}}}.\n"
                      "}}",
                      dataset_bounds.min.x, dataset_bounds.min.y, dataset_bounds.width(), dataset_bounds.height(),
                      dataset_largest_tile.zoom_level, dataset_largest_tile.coords.x, dataset_largest_tile.coords.y,
                      dataset_encompassing_tile.zoom_level, dataset_encompassing_tile.coords.x, dataset_encompassing_tile.coords.y,
                      tile_bounds.min.x, tile_bounds.min.y, tile_bounds.width(), tile_bounds.height(),
                      target_largest_tile.zoom_level, target_largest_tile.coords.x, target_largest_tile.coords.y,
                      target_encompassing_tile.zoom_level, target_encompassing_tile.coords.x, target_encompassing_tile.coords.y);
        } else if (error == mesh::BuildError::EmptyRegion) {
            LOG_ERROR("Target bounds are inside dataset, but the region is empty (only made up of padding)");
        }

        exit(1);
    }
    TerrainMesh mesh = mesh_result.value();
    LOG_DEBUG("Mesh building took {}s", format_secs_since(start));
    LOG_INFO("Finished building mesh geometry");

    if (texture_base_path.has_value()) {
        start = std::chrono::high_resolution_clock::now();
        LOG_INFO("Assembling mesh texture");
        BasemapSchemeTilePathProvider tile_provider(texture_base_path.value());
        std::optional<cv::Mat> texture = texture::assemble_texture_from_tiles(grid, texture_srs, texture_bounds, tile_provider);
        if (!texture.has_value()) {
            LOG_ERROR("Failed to assemble tile texture");
            exit(1);
        }
        mesh.texture = texture;
        LOG_DEBUG("Assembling mesh texture took {}s", format_secs_since(start));
        LOG_INFO("Finished assembling mesh texture");

    } else {
        LOG_INFO("Skipped assembling texture");
    }

    LOG_INFO("Writing mesh to output path {}", output_path.string());
    start = std::chrono::high_resolution_clock::now();
    // TODO: use a JSON libary instead
    std::unordered_map<std::string, std::string> metadata;
    metadata["mesh_srs"] = mesh_srs.GetAuthorityCode(nullptr);
    metadata["bounds_srs"] = tile_srs.GetAuthorityCode(nullptr);
    metadata["texture_srs"] = texture_srs.GetAuthorityCode(nullptr);
    metadata["tile_bounds"] = fmt::format(
        "{{ \"min\": {{ \"x\": {}, \"y\": {} }}, \"max\": {{ \"x\": {}, \"y\": {} }} }}",
        tile_bounds.min.x, tile_bounds.min.y, tile_bounds.max.x, tile_bounds.max.y);
    metadata["texture_bounds"] = fmt::format(
        "{{ \"min\": {{ \"x\": {}, \"y\": {} }}, \"max\": {{ \"x\": {}, \"y\": {} }} }}",
        texture_bounds.min.x, texture_bounds.min.y, texture_bounds.max.x, texture_bounds.max.y);
    if (!io::save_mesh_to_path(output_path, mesh, io::SaveOptions{.metadata = metadata}).has_value()) {
        LOG_ERROR("Failed to save mesh to file {}", output_path.string());
        exit(2);
    }
    LOG_DEBUG("Writing mesh took {}s", format_secs_since(start));
    LOG_INFO("Done", output_path.string());
}

}
