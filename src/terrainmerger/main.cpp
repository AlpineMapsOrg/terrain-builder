#include "cli.h"
#include "convert.h"
#include "log.h"
#include "merge.h"
#include "mesh/io.h"
#include "simplify.h"
#include "uv_map.h"
#include "validate.h"

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

        const TerrainMesh mesh = std::move(result.value());
        validate_mesh(mesh);
        validate_mesh(convert::mesh2cgal(mesh));
        meshes.push_back(mesh);
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

TerrainMesh simplify_mesh(const TerrainMesh &mesh, const cli::SimplificationArgs &args) {
    simplify::Options options{
        .stop_edge_ratio = args.factor,
        .error_bound = args.max_error};
    simplify::Result result = simplify::simplify_mesh(mesh, options);
    const TerrainMesh &simplified_mesh = result.mesh;

    const size_t initial_vertex_count = mesh.positions.size();
    const size_t initial_face_count = mesh.triangles.size();
    const size_t simplified_vertex_count = simplified_mesh.positions.size();
    const size_t simplified_face_count = simplified_mesh.triangles.size();

    LOG_DEBUG("Simplified mesh to {}/{} vertices and {}/{} faces",
              simplified_vertex_count, initial_vertex_count,
              simplified_face_count, initial_face_count);

    return result.mesh;
}

void run(const cli::Args &args) {
    LOG_INFO("Loading meshes...");
    std::vector<TerrainMesh> meshes = load_meshes_from_path(args.input_paths);

    const bool meshes_have_uvs = std::all_of(meshes.begin(), meshes.end(), [](const TerrainMesh &mesh) { return mesh.has_uvs(); });
    const bool meshes_have_textures = std::all_of(meshes.begin(), meshes.end(), [](const TerrainMesh &mesh) { return mesh.has_texture(); });

    LOG_INFO("Merging meshes...");
    merge::VertexMapping vertex_mapping;
    // TODO: adaptive merge distance or at least single connected component assert
    TerrainMesh merged_mesh = merge::merge_by_distance(meshes, 0.001, vertex_mapping);
    if (args.save_intermediate_meshes) {
        const std::filesystem::path merged_mesh_path = std::filesystem::path(args.output_path).replace_extension(".merged.glb");
        LOG_DEBUG("Saving merged mesh to {}", merged_mesh_path.string());
        io::save_mesh_to_path(merged_mesh_path, merged_mesh, io::SaveOptions{.name = "merged"});
    }

    if (meshes_have_uvs) {
        LOG_INFO("Calculating uv mapping...");
        const uv_map::UvMap uv_map = parameterize_mesh(merged_mesh);
        merged_mesh.uvs = uv_map::decode_uv_map(uv_map, merged_mesh.vertex_count());

        if (meshes_have_textures) {
            LOG_INFO("Merging textures...");
            merged_mesh.texture = uv_map::merge_textures(meshes, merged_mesh, vertex_mapping, uv_map, glm::uvec2(1024));
        }
    }

    TerrainMesh simplified_mesh;
    if (args.simplification) {
        LOG_INFO("Simplifying merged mesh...");
        simplified_mesh = simplify_mesh(merged_mesh, *args.simplification);

        if (merged_mesh.texture.has_value()) {
            LOG_INFO("Simplifying merged texture...");
            simplified_mesh.texture = simplify::simplify_texture(merged_mesh.texture.value(), glm::uvec2(256));
        }

        if (args.save_intermediate_meshes) {
            const std::filesystem::path simplified_mesh_path = std::filesystem::path(args.output_path).replace_extension(".simplified.glb");
            LOG_DEBUG("Saving simplified mesh to {}", simplified_mesh_path.string());
            io::save_mesh_to_path(simplified_mesh_path, simplified_mesh, io::SaveOptions{.name = "simplified"});
        }
    } else {
        simplified_mesh = merged_mesh;
    }

    LOG_INFO("Saving final mesh...");
    io::save_mesh_to_path(args.output_path, simplified_mesh);

    LOG_INFO("Done");
}

int main(int argc, char **argv) {
    const cli::Args args = cli::parse(argc, argv);
    Log::init(args.log_level);

    const std::string arg_str = std::accumulate(argv, argv + argc, std::string(),
                                                [](const std::string &acc, const char *arg) {
                                                    return acc + (acc.empty() ? "" : " ") + arg;
                                                });
    LOG_DEBUG("Running with: {}", arg_str);

    run(args);
}
