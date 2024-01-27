#ifndef TERRAINMESH_H
#define TERRAINMESH_H

#include <vector>
#include <optional>

#include <glm/glm.hpp>

#include "fi_image.h"

class TerrainMesh {
public:
    TerrainMesh(std::vector<glm::uvec3> triangles, std::vector<glm::dvec3> positions, std::vector<glm::dvec2> uvs) :
        TerrainMesh(triangles, positions, uvs, std::nullopt) {}
    TerrainMesh(std::vector<glm::uvec3> triangles, std::vector<glm::dvec3> positions, std::vector<glm::dvec2> uvs, FiImage texture)
        : TerrainMesh(triangles, positions, uvs, std::optional<FiImage>(std::move(texture))) {}
    TerrainMesh(std::vector<glm::uvec3> triangles, std::vector<glm::dvec3> positions, std::vector<glm::dvec2> uvs, std::optional<FiImage> texture)
        : triangles(triangles), positions(positions), uvs(uvs), texture(std::move(texture)) {}
    TerrainMesh() = default;
    TerrainMesh(TerrainMesh &&) = default;
    TerrainMesh &operator=(TerrainMesh &&) = default;

    std::vector<glm::uvec3> triangles;
    std::vector<glm::dvec3> positions;
    std::vector<glm::dvec2> uvs;
    std::optional<FiImage> texture;

    size_t vertex_count() const {
        return this->positions.size();
    }

    size_t face_count() const {
        return this->triangles.size();
    }

    bool has_uvs() const {
        return this->positions.size() == this->uvs.size();
    }
};

#endif
