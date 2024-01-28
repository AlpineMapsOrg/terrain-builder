#ifndef TERRAINMESH_H
#define TERRAINMESH_H

#include <vector>
#include <optional>

#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>

class TerrainMesh {
public:
    TerrainMesh(std::vector<glm::uvec3> triangles, std::vector<glm::dvec3> positions, std::vector<glm::dvec2> uvs) :
        TerrainMesh(triangles, positions, uvs, std::nullopt) {}
    TerrainMesh(std::vector<glm::uvec3> triangles, std::vector<glm::dvec3> positions, std::vector<glm::dvec2> uvs, cv::Mat texture)
        : TerrainMesh(triangles, positions, uvs, std::optional<cv::Mat>(std::move(texture))) {}
    TerrainMesh(std::vector<glm::uvec3> triangles, std::vector<glm::dvec3> positions, std::vector<glm::dvec2> uvs, std::optional<cv::Mat> texture)
        : triangles(triangles), positions(positions), uvs(uvs), texture(std::move(texture)) {}
    TerrainMesh() = default;
    TerrainMesh(TerrainMesh &&) = default;
    TerrainMesh &operator=(TerrainMesh &&) = default;
    TerrainMesh(const TerrainMesh &) = default;
    TerrainMesh &operator=(const TerrainMesh &) = default;

    std::vector<glm::uvec3> triangles;
    std::vector<glm::dvec3> positions;
    std::vector<glm::dvec2> uvs;
    std::optional<cv::Mat> texture;

    size_t vertex_count() const {
        return this->positions.size();
    }

    size_t face_count() const {
        return this->triangles.size();
    }

    bool has_uvs() const {
        return this->positions.size() == this->uvs.size();
    }
    bool has_texture() const {
        return this->texture.has_value();
    }
};

#endif
