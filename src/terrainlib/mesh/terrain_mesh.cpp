#include <ranges>

#include "terrain_mesh.h"

geometry::Aabb<3, double> calculate_bounds(const TerrainMesh& mesh) {
    geometry::Aabb<3, double> bounds;
    bounds.min = glm::dvec3(std::numeric_limits<double>::infinity());
    bounds.max = glm::dvec3(-std::numeric_limits<double>::infinity());
    for (unsigned int j = 0; j < mesh.positions.size(); j++) {
        const auto &position = mesh.positions[j];
        bounds.expand_by(position);
    }
    return bounds;
}

geometry::Aabb<3, double> calculate_bounds(std::span<const TerrainMesh> meshes) {
    geometry::Aabb<3, double> bounds;
    bounds.min = glm::dvec3(std::numeric_limits<double>::infinity());
    bounds.max = glm::dvec3(-std::numeric_limits<double>::infinity());
    for (unsigned int i = 0; i < meshes.size(); i++) {
        const TerrainMesh &mesh = meshes[i];
        for (unsigned int j = 0; j < mesh.positions.size(); j++) {
            const auto &position = mesh.positions[j];
            bounds.expand_by(position);
        }
    }
    return bounds;
}

std::vector<size_t> find_isolated_vertices(const TerrainMesh& mesh) {
    std::vector<bool> connected;
    connected.resize(mesh.vertex_count());
    std::fill(connected.begin(), connected.end(), false);
    for (const glm::uvec3 &triangle : mesh.triangles) {
        for (size_t k = 0; k < static_cast<size_t>(triangle.length()); k++) {
            connected[triangle[k]] = true;
        }
    }

    std::vector<size_t> isolated;
    for (size_t i = 0; i < mesh.vertex_count(); i++) {
        if (!connected[i]) {
            isolated.push_back(i);
        }
    }

    return isolated;
}

bool remove_isolated_vertices(TerrainMesh& mesh) {
    const std::vector<size_t> isolated = find_isolated_vertices(mesh);

    size_t removed_count = 0;
    std::vector<size_t> index_offset;
    for (size_t i : isolated | std::views::reverse) {
        const size_t last_index = mesh.positions.size() - 1;
        std::swap(mesh.positions[i], mesh.positions[last_index]);
        mesh.positions.pop_back();

        for (glm::uvec3 &triangle : mesh.triangles) {
            for (size_t k = 0; k < static_cast<size_t>(triangle.length()); k++) {
                if (triangle[k] == last_index) {
                    triangle[k] = i;
                }
            }
        }
    }

    return !isolated.empty();
}
