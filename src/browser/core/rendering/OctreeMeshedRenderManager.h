#pragma once
#include "GPUOctreeNode.h"
#include <cmrc/cmrc.hpp>
#include <core/Camera.h>
#include <core/shader/ShaderProgram.h>
#include <core/threading/SafeQueue.h>
#include <memory>
#include <mesh/SimpleMesh.h>
#include <shared_mutex>

class OctreeMeshedRenderManager
{
public:
    enum RenderMode
    {
        Wireframe,
        Textured,
        Textured_Unshaded,
        Clay,
        FlatNormals,
        UVs
    };

    std::shared_ptr<SafeQueue<std::tuple<octree::Id, std::shared_ptr<SimpleMesh>>>> p_mesh_queue;
    std::shared_ptr<SafeQueue<octree::Id>> p_removal_queue;

    OctreeMeshedRenderManager(octree::Space space);

    void set_render_mode(RenderMode new_render_mode);
    RenderMode get_render_mode();

    unsigned int get_meshed_node_count();

    void update(const std::shared_ptr<Camera> &camera);
    void render(const std::shared_ptr<Camera> &camera);

private:
    cmrc::embedded_filesystem RES;

    octree::Space m_space;

    RenderMode m_render_mode;

    std::unordered_map<octree::Id, std::unique_ptr<GPUOctreeNode>> m_meshed_node_drawlist;

    ShaderProgram sp_mesh;
    Shader vs_mesh;
    Shader fs_mesh;
    std::unique_ptr<Uniform<glm::mat4>> U_mesh_projection;
    std::unique_ptr<Uniform<glm::mat4>> U_mesh_view;
    std::unique_ptr<Uniform<glm::mat4>> U_mesh_model;
    std::unique_ptr<Uniform<float>> U_mesh_render_mode;
    std::unique_ptr<Uniform<int>> U_mesh_texture;
    std::unique_ptr<Uniform<bool>> U_mesh_has_texture;

    void initialize();
    void apply_render_mode();
};
