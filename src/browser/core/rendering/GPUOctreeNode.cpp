#include "GPUOctreeNode.h"
#include <glm/ext/matrix_transform.hpp>
#include <log.h>

// GPUOctreeNode::GPUOctreeNode(const SimpleMesh &mesh, const octree::Id &id, const octree::Space &space)
// {
//     m_original_node_center = space.get_node_bounds(id).centre();
//     m_camera_relative_node_center = m_original_node_center;
//     update_model_matrix();

//     if (mesh.positions.size() <= 0)
//     {
//         LOG_ERROR_AND_EXIT("Mesh of octree id {} has {} vertex positions!", id, mesh.positions.size());
//     }
//     if (mesh.triangles.size() <= 0)
//     {
//         LOG_ERROR_AND_EXIT("Mesh of octree id {} has {} triangles!", id, mesh.triangles.size());
//     }
//     // if (mesh.uvs.size() <= 0)
//     // {
//     //     LOG_ERROR_AND_EXIT("Mesh has {} uvs!", mesh.uvs.size());
//     // }

//     glGenVertexArrays(1, &m_vao_handle);
//     glBindVertexArray(m_vao_handle);

//     std::vector<glm::vec3> node_local_vertices;
//     node_local_vertices.reserve(mesh.positions.size());

//     for (const auto &pos : mesh.positions)
//     {
//         glm::vec3 node_local_pos = static_cast<glm::vec3>(pos - m_original_node_center);
//         node_local_vertices.push_back(node_local_pos);
//     }

//     std::vector<glm::vec2> float_uvs;
//     if (mesh.uvs.size() <= 0)
//     {

//         LOG_WARN("Mesh of octree id {} has {} uvs! Generating random uvs", id, mesh.uvs.size());
//         std::srand(std::time({}));
//         for (auto _ : mesh.positions)
//         {
//             float_uvs.push_back(glm::vec2(
//                 float(rand()) / float((RAND_MAX)),
//                 float(rand()) / float((RAND_MAX))));
//         }
//     }
//     else
//     {
//         for (const auto &uv : mesh.uvs)
//         {
//             glm::vec2 float_uv = static_cast<glm::vec2>(uv);
//             float_uvs.push_back(float_uv);
//         }
//     }

//     m_indices = std::make_unique<Buffer>(GL_ELEMENT_ARRAY_BUFFER, GL_STATIC_DRAW);
//     m_vertices = std::make_unique<Buffer>();
//     m_uvs = std::make_unique<Buffer>();

//     m_indices->set_data(mesh.triangles);
//     m_vertices->set_data(node_local_vertices);
//     m_uvs->set_data(float_uvs);
//     m_index_size = mesh.triangles.size() * 3;

//     m_vertices->bind();
//     glEnableVertexAttribArray(0);
//     glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), NULL);

//     m_uvs->bind();
//     glEnableVertexAttribArray(1);
//     glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), NULL);

//     m_indices->bind();

//     glBindVertexArray(0);

//     if (mesh.has_texture())
//     {
//         auto texture = mesh.texture.value();

//         // Texture
//         unsigned int tmp_tex_handle;
//         glGenTextures(1, &tmp_tex_handle);
//         glBindTexture(GL_TEXTURE_2D, tmp_tex_handle);

//         m_tex_handle = tmp_tex_handle;

//         // Ensure proper alignment (OpenCV width*channels might not match GL's default 4-byte pack)
//         glPixelStorei(GL_UNPACK_ALIGNMENT, (texture.step & 3) ? 1 : 4);
//         glPixelStorei(GL_UNPACK_ROW_LENGTH, texture.step / texture.elemSize());

//         glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//         glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

//         glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texture.cols, texture.rows, 0,
//                      GL_BGR, GL_UNSIGNED_BYTE, texture.data);
//     }
//     else
//     {
//         LOG_DEBUG("NO TEXTURE");
//     }
// }

GPUOctreeNode::GPUOctreeNode(const std::shared_ptr<SimpleMesh> &mesh, const octree::Id &id, const octree::Space &space)
{
    m_original_node_center = space.get_node_bounds(id).centre();
    m_camera_relative_node_center = m_original_node_center;
    update_model_matrix();

    if (mesh->positions.size() <= 0)
    {
        LOG_ERROR_AND_EXIT("Mesh of octree id {} has {} vertex positions!", id, mesh->positions.size());
    }
    if (mesh->triangles.size() <= 0)
    {
        LOG_ERROR_AND_EXIT("Mesh of octree id {} has {} triangles!", id, mesh->triangles.size());
    }
    // if (mesh.uvs.size() <= 0)
    // {
    //     LOG_ERROR_AND_EXIT("Mesh has {} uvs!", mesh.uvs.size());
    // }

    glGenVertexArrays(1, &m_vao_handle);
    glBindVertexArray(m_vao_handle);

    std::vector<glm::vec3> node_local_vertices;
    node_local_vertices.reserve(mesh->positions.size());

    for (const auto &pos : mesh->positions)
    {
        glm::vec3 node_local_pos = static_cast<glm::vec3>(pos - m_original_node_center);
        node_local_vertices.push_back(node_local_pos);
    }

    std::vector<glm::vec2> float_uvs;
    if (mesh->uvs.size() <= 0)
    {

        LOG_WARN("Mesh of octree id {} has {} uvs! Generating random uvs", id, mesh->uvs.size());
        std::srand(std::time({}));
        for (auto _ : mesh->positions)
        {
            float_uvs.push_back(glm::vec2(
                float(rand()) / float((RAND_MAX)),
                float(rand()) / float((RAND_MAX))));
        }
    }
    else
    {
        for (const auto &uv : mesh->uvs)
        {
            glm::vec2 float_uv = static_cast<glm::vec2>(uv);
            float_uvs.push_back(float_uv);
        }
    }

    m_indices = std::make_unique<Buffer>(GL_ELEMENT_ARRAY_BUFFER, GL_STATIC_DRAW);
    m_vertices = std::make_unique<Buffer>();
    m_uvs = std::make_unique<Buffer>();

    m_indices->set_data(mesh->triangles);
    m_vertices->set_data(node_local_vertices);
    m_uvs->set_data(float_uvs);
    m_index_size = mesh->triangles.size() * 3;

    m_vertices->bind();
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), NULL);

    m_uvs->bind();
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), NULL);

    m_indices->bind();

    glBindVertexArray(0);

    if (mesh->has_texture())
    {
        auto texture = mesh->texture.value();

        // Texture
        unsigned int tmp_tex_handle;
        glGenTextures(1, &tmp_tex_handle);
        glBindTexture(GL_TEXTURE_2D, tmp_tex_handle);

        m_tex_handle = tmp_tex_handle;

        // Ensure proper alignment (OpenCV width*channels might not match GL's default 4-byte pack)
        glPixelStorei(GL_UNPACK_ALIGNMENT, (texture.step & 3) ? 1 : 4);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, texture.step / texture.elemSize());

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texture.cols, texture.rows, 0,
                     GL_BGR, GL_UNSIGNED_BYTE, texture.data);
    }
    else
    {
        LOG_DEBUG("NO TEXTURE");
    }
}

void GPUOctreeNode::recenter(glm::dvec3 new_center)
{
    m_model_matrix_cache.reset();

    m_camera_relative_node_center = m_original_node_center - new_center;
}

glm::mat4 GPUOctreeNode::model_matrix()
{
    update_model_matrix();

    return m_model_matrix_cache.value();
}

void GPUOctreeNode::render(std::unique_ptr<Uniform<int>> &U_mesh_texture, std::unique_ptr<Uniform<bool>> &U_mesh_has_texture)
{
    if (m_tex_handle.has_value())
    {
        // LOG_DEBUG("Setting Texture: true");
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_tex_handle.value());
        U_mesh_texture->set(0);
        U_mesh_has_texture->set(true);
    }
    else
    {
        // LOG_DEBUG("Setting Texture: false");
        U_mesh_has_texture->set(false);
    }

    // LOG_DEBUG("Binding VAO");
    glBindVertexArray(m_vao_handle);
    // LOG_DEBUG("Rendering {} indices", m_index_size);
    glDrawElements(GL_TRIANGLES, m_index_size, GL_UNSIGNED_INT, 0);
    // LOG_DEBUG("Done rendering {} indices", m_index_size);

    if (m_tex_handle.has_value())
    {
        // LOG_DEBUG("Unbinding Texture");
        // Unbind texture
        glBindTexture(GL_TEXTURE_2D, 0);
    }
}

void GPUOctreeNode::update_model_matrix()
{
    if (!m_model_matrix_cache.has_value())
    {
        glm::mat4 translation = glm::translate(glm::dmat4(1.0f), m_camera_relative_node_center);
        m_model_matrix_cache.emplace(translation);
    }
}