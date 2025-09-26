#pragma once
#include "octree/Space.h"
#include <cmrc/cmrc.hpp>
#include <core/Buffer.h>
#include <core/Camera.h>
#include <core/process/param/SyncTrackedParam.h>
#include <core/shader/ShaderProgram.h>

class OctreeWireframeRenderManager
{
public:
    std::shared_ptr<SyncTrackedParam<std::vector<octree::Id>>> p_wireframe_ids;

    OctreeWireframeRenderManager(octree::Space space);

    unsigned int get_wireframe_node_count();
    float get_selection_opacity();
    void set_selection_opacity(float opacity);
    void set_selected_node(octree::Id id);
    std::optional<octree::Id> ray_cast_rendered_nodes(const std::shared_ptr<Camera> &camera);

    void update(const std::shared_ptr<Camera> &camera);
    void render(const std::shared_ptr<Camera> &camera);

private:
    cmrc::embedded_filesystem RES;

    octree::Space m_space;
    std::optional<octree::Id> m_selected_node;

    std::unique_ptr<Buffer> m_cube_vbo;

    // Selection Cube Data
    unsigned int m_selection_cube_vao;
    unsigned int m_selection_cube_indices_size;
    float m_selection_opacity;

    std::unique_ptr<Buffer> m_selection_cube_ibo;
    // Selection Box Data End

    // Wireframe Data
    unsigned int m_wireframe_cube_vao;
    unsigned int m_wireframe_cube_indices_size;

    std::unique_ptr<Buffer> m_wireframe_cube_ibo;
    std::unique_ptr<Buffer> m_wireframe_cube_instance_model_buffer;
    unsigned int m_wireframe_cube_instances;
    // Wireframe Data End

    ShaderProgram sp_wireframe;
    Shader vs_wireframe;
    Shader fs_wireframe;
    std::unique_ptr<Uniform<glm::mat4>> U_wireframe_view;
    std::unique_ptr<Uniform<glm::mat4>> U_wireframe_projection;

    ShaderProgram sp_selection;
    Shader vs_selection;
    std::unique_ptr<Uniform<glm::mat4>> U_selection_view;
    std::unique_ptr<Uniform<glm::mat4>> U_selection_projection;
    std::unique_ptr<Uniform<glm::mat4>> U_selection_cube_model;
    std::unique_ptr<Uniform<float>> U_selection_opacity;

    void initialize();
};
