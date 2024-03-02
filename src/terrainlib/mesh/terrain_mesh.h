#ifndef TERRAINMESH_H
#define TERRAINMESH_H

#include <vector>
#include <optional>
#include <span>

#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>
#include <radix/geometry.h>
#include <zpp_bits.h>

class TerrainMesh {
public:
    using serialize = zpp::bits::members<4>;

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

geometry::Aabb<3, double> calculate_bounds(const TerrainMesh &mesh);
geometry::Aabb<3, double> calculate_bounds(std::span<const TerrainMesh> meshes);

std::vector<size_t> find_isolated_vertices(const TerrainMesh& mesh);
size_t remove_isolated_vertices(TerrainMesh& mesh);
size_t remove_triangles_of_negligible_size(TerrainMesh& mesh, const double threshold_percentage_of_average = 0.001);

void sort_triangles(TerrainMesh &mesh);
void sort_triangles(std::span<glm::uvec3> triangles);

void validate_mesh(const TerrainMesh &mesh);

#endif
