#include "cli.h"
#include "io.h"
#include "merge.h"
#include "simplify.h"
#include "uv_map.h"
#include "convert.h"
#include "log.h"

#include <CGAL/boost/graph/IO/OBJ.h>

#ifndef DEBUG
#define DEBUG !NDEBUG
#endif

std::vector<TerrainMesh> load_meshes_from_path(std::span<const std::filesystem::path> paths, const bool print_errors = true) {
    std::vector<TerrainMesh> meshes;
    meshes.reserve(paths.size());
    for (const std::filesystem::path &path : paths) {
        auto result = io::load_mesh_from_path(path);
        if (!result.has_value()) {
            const io::LoadMeshError error = result.error();
            if (print_errors) {
                LOG_ERROR("Failed to load mesh from {}: {}", path.string(), error.description());
            }
            exit(EXIT_FAILURE);
        }
        
        TerrainMesh mesh = std::move(result.value());
        meshes.push_back(std::move(mesh));
    }

    return meshes;
}

uv_map::UvMap parameterize_mesh(TerrainMesh &mesh) {
    const tl::expected<uv_map::UvMap, uv_map::UvParameterizationError> result =
        uv_map::parameterize_mesh(mesh, uv_map::Algorithm::DiscreteConformalMap, uv_map::Border::Circle);
    if (result) {
        return result.value();
    } else {
        const uv_map::UvParameterizationError error = result.error();
        LOG_ERROR("Failed to parameterize merged mesh due to '{}'", error.description());
        exit(EXIT_FAILURE);
    }
}

void run(const cli::Args &args) {
    Log::init(args.log_level);

    LOG_INFO("Loading meshes...");
    std::vector<TerrainMesh> meshes = load_meshes_from_path(args.input_paths);

    LOG_INFO("Merging meshes...");
    merge::VertexMapping vertex_mapping;
    TerrainMesh merged_mesh = merge::merge_by_distance(meshes, 0.1, vertex_mapping);

    LOG_INFO("Calculating uv mapping...");
    const uv_map::UvMap uv_map = parameterize_mesh(merged_mesh);
    merged_mesh.uvs = uv_map::decode_uv_map(uv_map, merged_mesh.vertex_count());

    LOG_INFO("Merging textures...");
    merged_mesh.texture = uv_map::merge_textures(meshes, merged_mesh, vertex_mapping, uv_map, glm::uvec2(1024));

    TerrainMesh simplified_mesh;
    if (args.simplification) {
        LOG_INFO("Simplifying merged mesh...");
        simplified_mesh = simplify_mesh(merged_mesh, *args.simplification);

        LOG_INFO("Simplifying merged texture...");
        simplified_mesh.texture = simplify::simplify_texture(merged_mesh.texture.value(), glm::uvec2(256));
    } else {
        simplified_mesh = merged_mesh;
    }

    LOG_INFO("Saving final mesh...");
    io::save_mesh_to_path(args.output_path, simplified_mesh);
}

int main(int argc, char **argv) {
    const std::array<std::string, 10> raw_args = {"./terrainmerger", "--input", "183325_276252.glb", "183325_276253.glb", "183326_276252.glb", "183326_276253.glb", "--output", "./out5.glb", "--verbosity", "trace"};
    const std::span<const std::string> raw_args_span = raw_args;
    const cli::Args args = cli::parse(raw_args_span);
    run(args);
}
