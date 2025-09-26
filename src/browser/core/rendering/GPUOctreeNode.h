#pragma once
#include <core/Buffer.h>
#include <core/shader/Uniform.h>
#include <mesh/SimpleMesh.h>
#include <octree/Id.h>
#include <octree/Space.h>
#include <octree/Storage.h>

class GPUOctreeNode
{
public:
    // GPUOctreeNode(const SimpleMesh &mesh, const octree::Id &id, const octree::Space &space);
    GPUOctreeNode(const std::shared_ptr<SimpleMesh> &mesh, const octree::Id &id, const octree::Space &space);

    void recenter(glm::dvec3 new_center);
    glm::mat4 model_matrix();
    void render(std::unique_ptr<Uniform<int>> &U_mesh_texture, std::unique_ptr<Uniform<bool>> &U_mesh_has_texture);

private:
    unsigned int m_vao_handle;

    glm::dvec3 m_original_node_center;
    glm::dvec3 m_camera_relative_node_center;
    std::optional<glm::mat4> m_model_matrix_cache;

    size_t m_index_size;

    std::unique_ptr<Buffer> m_indices;
    std::unique_ptr<Buffer> m_vertices;
    std::unique_ptr<Buffer> m_uvs;

    std::optional<unsigned int> m_tex_handle;

    GPUOctreeNode();

    void update_model_matrix();
};