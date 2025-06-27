#pragma once
#include "GPUOctreeNode.h"
#include "octree/Id.h"
#include "octree/Space.h"
#include "octree/Storage.h"
#include <any>
#include <core/io/OctreeNodeRepository.h>
#include <core/shader/Uniform.h>
#include <glm/glm.hpp>
#include <unordered_map>
// CMRC Resource Compiler
#include "core/Camera.h"
#include "core/shader/Shader.h"
#include "core/shader/ShaderProgram.h"
#include <cmrc/cmrc.hpp>

namespace octree
{

    struct OctreeRenderIntent
    {
        std::vector<float> instances_active;
        std::vector<glm::mat4> instances_model_mats;
        size_t instance_count;

        std::optional<double> min_scene_distance;
        std::optional<double> max_scene_distance;

        std::optional<Id> closest_node;
    };

    enum OctreeFilterParamType
    {
        Float,
        Double
    };

    struct OctreeFilterParam
    {
        std::string name;
        OctreeFilterParamType type;
        std::any default_value;
        std::string description;
    };

    struct OctreeFilterDefinition
    {
        std::string name;
        std::string description;
    };

    enum RenderMode
    {
        Wireframe,
        Textured,
        Clay,
        FlatNormals
    };

    class OctreeRenderManager
    {
    public:
        OctreeRenderManager(std::shared_ptr<OctreeNodeRepository> octree_node_repository, Space space);

        void set_render_mode(RenderMode new_render_mode);
        RenderMode get_render_mode();
        void set_selected_node(octree::Id id);
        std::optional<octree::Id> ray_cast_rendered_nodes(const std::shared_ptr<Camera> &camera);

        void update(const std::shared_ptr<Camera> &camera);
        void render();

        int get_last_node_draw_amount();

        // void populate_gpu_nodes(glm::dvec3 cam_pos);
        // OctreeRenderIntent generate_visible_octree_nodes(glm::dvec3 cam_pos);
        // void render_gpu_nodes(Uniform<glm::mat4> U_model_mesh, glm::dvec3 cam_pos);
        // OctreeRenderIntent generate_octree_render_intent(const Id root, glm::dvec3 cam_pos, bool draw_neighbours_only, float refining_ratio);

        std::unique_ptr<Uniform<glm::mat4>> U_projection;

    private:
        RenderMode m_render_mode;

        cmrc::embedded_filesystem RES;

        Space m_space;
        // std::shared_ptr<Storage> m_storage;
        std::shared_ptr<OctreeNodeRepository> m_repository;

        const size_t m_max_rendered_nodes = 15;
        const size_t m_max_node_candidates = 200;

        std::optional<octree::Id> m_selected_node;

        std::vector<std::shared_ptr<GPUOctreeNode>> m_octree_mesh_node_drawlist;
        std::vector<Id> m_octree_node_drawlist;

        unsigned int m_node_cube_vao;
        unsigned int m_cube_line_indices_size;
        unsigned int m_cube_instances_active;
        std::unique_ptr<Buffer> cube_ibo;
        std::unique_ptr<Buffer> cube_vbo;
        std::unique_ptr<Buffer> cube_instance_active_buffer;
        std::unique_ptr<Buffer> cube_instance_model_buffer;

        ShaderProgram sp_octree_lines;
        Shader vs_octree_lines;
        Shader fs_octree_lines;
        std::unique_ptr<Uniform<glm::mat4>> U_view;

        ShaderProgram sp_octree_mesh;
        Shader vs_octree_mesh;
        Shader fs_octree_mesh;
        std::unique_ptr<Uniform<glm::mat4>> U_mesh_projection;
        std::unique_ptr<Uniform<glm::mat4>> U_mesh_view;
        std::unique_ptr<Uniform<glm::mat4>> U_mesh_model;
        std::unique_ptr<Uniform<float>> U_mesh_render_mode;
        std::unique_ptr<Uniform<int>> U_mesh_texture;

        void init_node_cube_rendering();
        void init_node_mesh_rendering();
    };

}