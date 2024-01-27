#ifndef MERGE_H
#define MERGE_H

#include <span>
#include <vector>
#include <unordered_map>

#include <radix/geometry.h>
#include <glm/glm.hpp>

#include "terrain_mesh.h"

namespace merge {

struct VertexId {
    size_t mesh_index;
    size_t vertex_index;
};

struct TriangleInMesh {
    size_t mesh_index;
    glm::uvec3 triangle;
};

class VertexMapping {
public:
    void init(std::span<const size_t> vertex_counts) {
        this->forward.resize(vertex_counts.size());
        for (size_t i = 0; i < vertex_counts.size(); i++) {
            this->forward[i].resize(vertex_counts[i]);
        }

        this->backward.resize(vertex_counts.size());
        for (size_t i = 0; i < vertex_counts.size(); i++) {
            this->backward[i].reserve(vertex_counts[i]);
        }
    }

    void add_bidirectional(VertexId source, size_t mapped) {
        this->add_forward(source, mapped);
        this->add_backward(source, mapped);
    }

    void add_forward(VertexId source, size_t mapped) {
        this->forward[source.mesh_index][source.vertex_index] = mapped;
    }

    void add_backward(VertexId source, size_t mapped) {
        this->backward[source.mesh_index][mapped] = source.vertex_index;
    }

    size_t map(VertexId source) const {
        return this->forward.at(source.mesh_index).at(source.vertex_index);
    }

    std::optional<size_t> map_inverse(size_t mesh_index, size_t mapped_index) const {
        const auto it = this->backward.at(mesh_index).find(mapped_index);
        if (it != this->backward.at(mesh_index).end()) {
            return it->second;
        }
        return std::nullopt;
    }

    std::vector<size_t> map_inverse_exists(size_t mapped_index) const {
        std::vector<size_t> exists;
        exists.reserve(this->mesh_count());
        for (size_t mesh_index = 0; mesh_index < this->mesh_count(); mesh_index++) {
            if (this->map_inverse(mapped_index, mesh_index)) {
                exists.push_back(mesh_index);
            }
        }
        return exists;
    }

    TriangleInMesh find_source_triangle(glm::uvec3 mapped_triangle) const {
        for (size_t mesh_index = 0; mesh_index < this->mesh_count(); mesh_index++) {
            const std::optional<glm::uvec3> source_triangle_opt = this->find_source_triangle_in_mesh(mapped_triangle, mesh_index);
            if (source_triangle_opt.has_value()) {
                const glm::uvec3 source_triangle = source_triangle_opt.value();
                return TriangleInMesh { .mesh_index=mesh_index, .triangle=source_triangle };
            }
        }

        throw std::runtime_error("illegal state in vertex mapping");
    }

    std::optional<glm::uvec3> find_source_triangle_in_mesh(glm::uvec3 mapped_triangle, size_t mesh_index) const {
        glm::uvec3 source_triangle;

        for (size_t i = 0; i < mapped_triangle.length(); i++) {
            const size_t mapped_vertex_index = mapped_triangle[i];
            const std::optional<size_t> source_vertex = this->map_inverse(mesh_index, mapped_vertex_index);
            if (source_vertex.has_value()) {
                source_triangle[i] = source_vertex.value();
            } else {
                return std::nullopt;
            }

            assert(this->map(VertexId { .mesh_index = mesh_index, .vertex_index = source_vertex.value() }) == mapped_vertex_index);
        }

        return source_triangle;
    }

    size_t mesh_count() const {
        return this->backward.size();
    }

    void validate() const {
        for (size_t i = 0; i < this->mesh_count(); i++) {
            assert(this->forward[i].size() == this->backward[i].size());

            for (size_t j = 0; j < this->forward[i].size(); j++) {
                const size_t mapped = this->map(VertexId { .mesh_index = i, .vertex_index = j });
                const std::optional<size_t> inv_mapped = this->map_inverse(i, mapped);
                assert(inv_mapped.has_value());
                assert(inv_mapped.value() == j);
            }

            for (const std::pair<unsigned int, unsigned int> e : this->backward[i]) {
                const std::optional<size_t> inv_mapped = this->map_inverse(i, e.first);
                assert(inv_mapped.has_value());
                const size_t mapped = this->map(VertexId{.mesh_index = i, .vertex_index = inv_mapped.value()});
                assert(mapped == e.first);
            }
        }
    }

    private:
        std::vector<std::vector<size_t>> forward;
        std::vector<std::unordered_map<size_t, size_t>> backward;
};

VertexMapping create_merge_mapping(std::span<const TerrainMesh> meshes, double distance_epsilon);

TerrainMesh merge_mased_on_mapping(std::span<const TerrainMesh> meshes, const VertexMapping &mapping);

TerrainMesh merge_by_distance(std::span<const TerrainMesh> meshes, double distance_epsilon, VertexMapping &mapping);
TerrainMesh merge_by_distance(std::span<const TerrainMesh> meshes, double distance_epsilon);

}

#endif
