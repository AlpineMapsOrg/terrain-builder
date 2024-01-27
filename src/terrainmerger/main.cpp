#include "cli.h"
#include "io.h"
#include "merge.h"
#include "simplify.h"
#include "uv_map.h"
#include "convert.h"

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
                fmt::println(stderr, "Failed to load mesh from {}: {}", path.string(), error.description());
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
        fmt::println(stderr, "Failed to parameterize merged mesh due to '{}'", error.description());
        exit(EXIT_FAILURE);
    }
}

void run(const cli::Args &args) {
    fmt::println("Loading meshes...");
    std::vector<TerrainMesh> meshes = load_meshes_from_path(args.input_paths);

    fmt::println("Merging meshes...");
    merge::VertexMapping vertex_mapping;
    TerrainMesh merged_mesh = merge::merge_by_distance(meshes, 0.1, vertex_mapping);

    fmt::println("Calculating uv mapping...");
    const uv_map::UvMap uv_map = parameterize_mesh(merged_mesh);
    merged_mesh.uvs = uv_map::decode_uv_map(uv_map, merged_mesh.vertex_count());

    fmt::println("Merging textures...");
    merged_mesh.texture = convert::cv2fi(uv_map::merge_textures(meshes, merged_mesh, vertex_mapping, uv_map, glm::uvec2(1024)));

    fmt::println("Simplifying merged mesh...");
    TerrainMesh simplified_mesh = simplify::simplify_mesh(merged_mesh);

    fmt::println("Simplifying merged texture...");
    simplified_mesh.texture = simplify::simplify_texture(merged_mesh.texture.value(), glm::uvec2(256));

    fmt::println("Saving final mesh...");
    io::save_mesh_to_path(args.output_path, simplified_mesh);
}

int main(int argc, char **argv) {
    // Override argc and argv
    argc = 8; // Set the desired number of arguments
    char *new_argv[] = {"./terrainmerger", "--input", "183325_276252.glb", "183325_276253.glb", "183326_276252.glb", "183326_276253.glb", "--output", "./out5.glb"};
    argv = new_argv;

    const cli::Args args = cli::parse(argc, argv);
    run(args);
}
