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