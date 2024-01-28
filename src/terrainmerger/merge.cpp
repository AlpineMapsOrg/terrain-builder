#include <span>
#include <unordered_map>
#include <vector>

#include <glm/glm.hpp>
#include <radix/geometry.h>

#include "terrain_mesh.h"
#include "merge.h"
#include "log.h"

using namespace merge;

template <typename T>
class Grid3d {
public:
    Grid3d(const glm::dvec3 origin, const glm::dvec3 size, const glm::uvec3 divisions)
        : origin(origin), size(size), divisions(divisions) {
        grid_data.resize(divisions.x * divisions.y * divisions.z);
    }

    struct GridCellItem {
        glm::dvec3 point;
        T value;
    };

    struct GridCell {
        std::vector<GridCellItem> items;
    };

    std::optional<std::reference_wrapper<T>> find(const glm::dvec3 point, const double epsilon = 0.1f) {
        if (!is_in_bounds(point)) {
            return std::nullopt;
        }

        const size_t cell_index = this->calculate_cell_index(point);

        GridCell &cell = this->grid_data[cell_index];
        std::optional<std::reference_wrapper<T>> closest_value;
        double closest_distance = std::numeric_limits<double>::infinity();
        for (GridCellItem &item : cell.items) {
            const double distance = glm::distance(point, item.point);
            if (distance < epsilon && distance < closest_distance) {
                closest_value = item.value;
                closest_distance = distance;
            }
        }

        // TODO: check if point + epsilon is in neighbouring cells
        return closest_value;
    }

    std::optional<std::reference_wrapper<const T>> find(const glm::dvec3 point, const double epsilon = 0.1f) const {
        return const_cast<std::optional<std::reference_wrapper<const T>>>(
            const_cast<const Grid3d *>(this)->find(point, epsilon));
    }

    void insert(const glm::dvec3 point, const T value) {
        if (!is_in_bounds(point)) {
            return;
        }

        const size_t cell_index = this->calculate_cell_index(point);

        const GridCellItem item{
            .point = point,
            .value = value};
        GridCell &cell = grid_data[cell_index];
        cell.items.push_back(item);
    }

    bool is_in_bounds(const glm::dvec3 point) const {
        const glm::dvec3 max_bounds = origin + size;
        return glm::all(glm::greaterThanEqual(point, origin)) && glm::all(glm::lessThanEqual(point, max_bounds));
    }

    size_t calculate_cell_index(const glm::dvec3 point) const {
        const glm::dvec3 cell_size = this->size / glm::dvec3(this->divisions);
        const glm::uvec3 cell_index = glm::uvec3((point - this->origin) / cell_size);
        return cell_index.x + cell_index.y * divisions.x + cell_index.z * divisions.x * divisions.y;
    }

private:
    const glm::dvec3 origin;
    const glm::dvec3 size;
    const glm::uvec3 divisions;
    std::vector<GridCell> grid_data;
};

static geometry::Aabb<3, double> calculate_bounds(std::span<const TerrainMesh> meshes) {
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

static geometry::Aabb<3, double> pad_bounds(const geometry::Aabb<3, double> &bounds, double percentage) {
    const glm::dvec3 bounds_padding = bounds.size() * percentage;
    const geometry::Aabb<3, double> padded_bounds(bounds.min - bounds_padding, bounds.max + bounds_padding);
    return padded_bounds;
}

VertexMapping merge::create_merge_mapping(std::span<const TerrainMesh> meshes, double distance_epsilon) {
    const geometry::Aabb<3, double> bounds = calculate_bounds(meshes);
    const geometry::Aabb<3, double> padded_bounds = pad_bounds(bounds, 0.01);

    std::vector<size_t> mesh_sizes;
    mesh_sizes.reserve(meshes.size());
    std::transform(meshes.begin(), meshes.end(),
                   std::back_inserter(mesh_sizes),
                   [](const TerrainMesh &mesh) { return mesh.vertex_count(); });

    VertexMapping mapping;
    mapping.init(mesh_sizes);

    // TODO: make subdivisions dependent on relative bounds extends
    Grid3d<VertexId> grid(padded_bounds.min, padded_bounds.size(), glm::uvec3(100));
    size_t unique_vertices = 0;
    for (size_t mesh_index = 0; mesh_index < meshes.size(); mesh_index++) {
        const TerrainMesh &mesh = meshes[mesh_index];
        for (size_t vertex_index = 0; vertex_index < mesh.vertex_count(); vertex_index++) {
            const glm::dvec3 &position = mesh.positions[vertex_index];
            const VertexId current_vertex{
                .mesh_index = mesh_index,
                .vertex_index = vertex_index};

            const std::optional<std::reference_wrapper<VertexId>> other_vertex = grid.find(position, distance_epsilon);
            if (other_vertex.has_value()) {
                // duplicated
                mapping.add_bidirectional(current_vertex, mapping.map(other_vertex.value()));
            } else {
                // TODO: add mapped index instead
                grid.insert(position, current_vertex);
                mapping.add_bidirectional(current_vertex, unique_vertices);

                unique_vertices += 1;
            }
        }
    }

#if DEBUG
    mapping.validate();
#endif

    return mapping;
}

TerrainMesh merge::merge_mased_on_mapping(std::span<const TerrainMesh> meshes, const merge::VertexMapping &mapping) {
    TerrainMesh merged_mesh;

    size_t max_combined_vertex_count = 0;
    size_t max_combined_face_count = 0;
    for (const TerrainMesh &mesh : meshes) {
        max_combined_vertex_count += mesh.vertex_count();
        max_combined_face_count += mesh.face_count();
    }

    const bool has_uvs = std::all_of(meshes.begin(), meshes.end(), [](const TerrainMesh &mesh) {
        return mesh.has_uvs();
    });

    size_t max_vertex_index = 0;
    merged_mesh.positions.resize(max_combined_vertex_count);
    if (has_uvs) {
        merged_mesh.uvs.resize(max_combined_vertex_count);
    }
    for (size_t mesh_index = 0; mesh_index < meshes.size(); mesh_index++) {
        const TerrainMesh &mesh = meshes[mesh_index];
        for (size_t vertex_index = 0; vertex_index < mesh.vertex_count(); vertex_index++) {
            const size_t mapped_index = mapping.map(VertexId{.mesh_index = mesh_index, .vertex_index = vertex_index});
            merged_mesh.positions[mapped_index] = mesh.positions[vertex_index];
            if (has_uvs) {
                merged_mesh.uvs[mapped_index] = mesh.uvs[vertex_index];
            }
            max_vertex_index = std::max(max_vertex_index, mapped_index);
        }
    }
    assert(max_vertex_index < merged_mesh.vertex_count());
    merged_mesh.positions.resize(max_vertex_index + 1);
    if (has_uvs) {
        merged_mesh.uvs.resize(max_vertex_index + 1);
    }

    merged_mesh.triangles.reserve(max_combined_face_count);
    for (size_t mesh_index = 0; mesh_index < meshes.size(); mesh_index++) {
        const TerrainMesh &mesh = meshes[mesh_index];
        for (size_t triangle_index = 0; triangle_index < mesh.face_count(); triangle_index++) {
            const glm::uvec3 &triangle = mesh.triangles[triangle_index];

            glm::uvec3 new_triangle;
            for (size_t k = 0; k < static_cast<size_t>(triangle.length()); k++) {
                new_triangle[k] = mapping.map(VertexId{.mesh_index = mesh_index, .vertex_index = triangle[k]});
            }
            if (new_triangle[0] == new_triangle[1] ||
                new_triangle[1] == new_triangle[2] ||
                new_triangle[2] == new_triangle[0]) {
                LOG_INFO("Skipping illegal triangle...");
                continue;
            }

            merged_mesh.triangles.push_back(new_triangle);
        }
    }

    return merged_mesh;
}

TerrainMesh merge::merge_by_distance(std::span<const TerrainMesh> meshes, double distance_epsilon, merge::VertexMapping &mapping) {
    mapping = merge::create_merge_mapping(meshes, distance_epsilon);
    return merge::merge_mased_on_mapping(meshes, mapping);
}
TerrainMesh merge::merge_by_distance(std::span<const TerrainMesh> meshes, double distance_epsilon) {
    VertexMapping mapping;
    return merge::merge_by_distance(meshes, distance_epsilon, mapping);
}
