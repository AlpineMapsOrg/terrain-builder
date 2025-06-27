#include "OctreeRenderManager.h"
#include <core/Camera.h>
#include <glm/gtc/matrix_transform.hpp>
#include <string>

#include <core/geometry/RadixExtensions.h>
#include <core/geometry/UnitCube.h>
#include <core/shader/Shader.h>
#include <core/shader/ShaderProgram.h>
#include <set>

CMRC_DECLARE(res);

namespace octree
{
    OctreeRenderManager::OctreeRenderManager(std::shared_ptr<OctreeNodeRepository> octree_node_repository, Space space)
        : RES(cmrc::res::get_filesystem()),
          m_space(space),
          m_repository(octree_node_repository),
          vs_octree_lines(GL_VERTEX_SHADER), fs_octree_lines(GL_FRAGMENT_SHADER),
          vs_octree_mesh(GL_VERTEX_SHADER), fs_octree_mesh(GL_FRAGMENT_SHADER)
    {
        init_node_cube_rendering();
        init_node_mesh_rendering();

        set_render_mode(RenderMode::Clay);
    }

    void OctreeRenderManager::set_render_mode(RenderMode new_render_mode)
    {
        m_render_mode = new_render_mode;

        sp_octree_mesh.use();
        if (m_render_mode == RenderMode::Wireframe)
        {
            glDisable(GL_CULL_FACE);
            // glCullFace(GL_BACK);
            glCullFace(GL_FRONT_AND_BACK);

            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

            U_mesh_render_mode->set(0);
        }
        else if (m_render_mode == RenderMode::Textured)
        {
            glDisable(GL_CULL_FACE);
            // glCullFace(GL_BACK);

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            U_mesh_render_mode->set(1);
        }
        else if (m_render_mode == RenderMode::Clay)
        {
            glDisable(GL_CULL_FACE);
            // glCullFace(GL_BACK);

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            U_mesh_render_mode->set(2);
        }
        else if (m_render_mode == RenderMode::FlatNormals)
        {
            glDisable(GL_CULL_FACE);
            // glCullFace(GL_BACK);

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            U_mesh_render_mode->set(3);
        }
        else if (m_render_mode == RenderMode::UVs)
        {
            glDisable(GL_CULL_FACE);
            // glCullFace(GL_BACK);

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            U_mesh_render_mode->set(4);
        }
    }

    RenderMode OctreeRenderManager::get_render_mode()
    {
        return m_render_mode;
    }

    void OctreeRenderManager::set_selected_node(octree::Id id)
    {
        m_selected_node = id;
    }

    std::optional<octree::Id> OctreeRenderManager::ray_cast_rendered_nodes(const std::shared_ptr<Camera> &camera)
    {
        glm::dvec3 ray_dir = camera->get_local_forward_dir();
        glm::dvec3 ray_start = camera->get_position();

        std::optional<octree::Id> closest_id;
        double closest_t = std::numeric_limits<double>::infinity();

        for (octree::Id &id : m_octree_node_drawlist)
        {
            octree::Bounds bounds = m_space.get_node_bounds(id);

            // If we're inside a node, just return that node
            if (bounds.contains(ray_start))
            {
                return id;
            }

            glm::dvec2 itx = radix::geometry::ray_intersect(bounds, ray_start, ray_dir);

            // Check if valid intersection
            if (itx.x <= itx.y && itx.x < closest_t)
            {
                closest_id = id;
                closest_t = itx.x;
            }
        }

        return closest_id;
    }

    void OctreeRenderManager::update(const std::shared_ptr<Camera> &camera)
    {
        auto cam_pos = camera->get_position();

        m_octree_node_drawlist.clear();

        std::vector<Id> id_traverse_queue;
        id_traverse_queue.push_back(octree::Id::root());

        std::vector<float> instance_active;
        std::vector<glm::mat4> instance_models;

        while (!id_traverse_queue.empty() && m_octree_node_drawlist.size() < m_max_node_candidates)
        {
            Id current = id_traverse_queue.back();
            id_traverse_queue.pop_back();

            if (!m_repository->has_node(current))
            {
                continue;
            }

            auto status = m_repository->get_node_status(current);

            if (status.has_value() && status.value() == NodeStatus::Virtual)
            {
                // LOG_DEBUG("Node {} is VIRTUAL splitting", current);

                // Split node into 8 children and add them to the refining list
                std::array<octree::Id, 8> children = current.children().value();

                for (Id child : children)
                {
                    id_traverse_queue.push_back(child);
                }

                // Sort the whole queue based on the distance to the camera
                std::sort(id_traverse_queue.begin(), id_traverse_queue.end(), [this, cam_pos](octree::Id a, octree::Id b)
                          {
                            auto a_dist = glm::distance(m_space.get_node_bounds(a).centre(), cam_pos);
                            auto b_dist = glm::distance(m_space.get_node_bounds(b).centre(), cam_pos);

                            return a_dist < b_dist; });
            }
            else if (status.has_value() && status.value() == NodeStatus::Inner)
            {
                // LOG_DEBUG("Node {} is INNER", current);
            }
            else if (status.has_value() && status.value() == NodeStatus::Leaf)
            {
                // LOG_DEBUG("Node {} is LEAF", current);

                m_octree_node_drawlist.push_back(current);
            }
        }

        // Add nodes from direct file sources
        auto registered_files = m_repository->get_registered_file_ids();
        m_octree_node_drawlist.insert(m_octree_node_drawlist.end(), registered_files.begin(), registered_files.end());

        // Remove duplicates
        std::set<octree::Id> s(m_octree_node_drawlist.begin(), m_octree_node_drawlist.end());
        m_octree_node_drawlist.assign(s.begin(), s.end());

        // Sort the whole candidate list based on the distance to the camera
        std::sort(m_octree_node_drawlist.begin(), m_octree_node_drawlist.end(), [this, cam_pos](octree::Id a, octree::Id b)
                  {
            auto a_dist = glm::distance(m_space.get_node_bounds(a).centre(), cam_pos);
            auto b_dist = glm::distance(m_space.get_node_bounds(b).centre(), cam_pos);
            
            return a_dist < b_dist; });

        // Truncate the candidates to the max allowed render count
        if (m_octree_node_drawlist.size() > m_max_rendered_nodes)
        {
            m_octree_node_drawlist.resize(m_max_rendered_nodes);
        }

        for (auto &id : m_octree_node_drawlist)
        {
            auto bounds = m_space.get_node_bounds(id);

            glm::dmat4 model_scale = glm::scale(glm::dmat4(1.0f), bounds.size());
            glm::dmat4 model_translate = glm::translate(glm::dmat4(1.0f), bounds.centre() - cam_pos);
            glm::mat4 model = model_translate * model_scale;

            if (id == m_selected_node)
            {
                instance_active.push_back(1.0f);
            }
            else
            {
                instance_active.push_back(0.0f);
            }

            instance_models.push_back(glm::mat4(model));
        }

        m_cube_instances_active = instance_active.size();

        cube_instance_active_buffer->set_data(instance_active);
        cube_instance_model_buffer->set_data(instance_models);

        m_octree_mesh_node_drawlist = m_repository->load_nodes(m_octree_node_drawlist, m_space);

        if (!m_octree_mesh_node_drawlist.empty())
        {
            for (std::shared_ptr<GPUOctreeNode> &n : m_octree_mesh_node_drawlist)
            {
                n->recenter(cam_pos);
            }
        }

        // Update Camera Matrices & Recenter GPUNodes
        if (camera->is_view_matrix_outdated())
        {
            sp_octree_lines.use();
            U_view->set(camera->view_matrix());
            sp_octree_mesh.use();
            U_mesh_view->set(camera->view_matrix());
        }
        if (camera->is_projection_matrix_outdated())
        {
            sp_octree_lines.use();
            U_projection->set(camera->projection_matrix());
            sp_octree_mesh.use();
            U_mesh_projection->set(camera->projection_matrix());
        }
    }

    void OctreeRenderManager::render()
    {
        sp_octree_mesh.use();
        for (std::shared_ptr<GPUOctreeNode> &n : m_octree_mesh_node_drawlist)
        {
            U_mesh_model->set(n->model_matrix());
            n->render(U_mesh_texture, U_mesh_has_texture);
        }

        sp_octree_lines.use();
        glBindVertexArray(m_node_cube_vao);
        glDrawElementsInstanced(GL_LINES, m_cube_line_indices_size, GL_UNSIGNED_INT, 0, m_cube_instances_active);
    }

    int OctreeRenderManager::get_last_node_draw_amount()
    {
        return m_octree_node_drawlist.size();
    }

    void OctreeRenderManager::init_node_cube_rendering()
    {
        auto vsc_octree_lines = RES.open("shaders/octree_lines.vert");
        vs_octree_lines.compile(std::string_view(vsc_octree_lines.begin(), vsc_octree_lines.end()));

        auto fsc_octree_lines = RES.open("shaders/octree_lines.frag");
        fs_octree_lines.compile(std::string_view(fsc_octree_lines.begin(), fsc_octree_lines.end()));

        sp_octree_lines.attach(vs_octree_lines);
        sp_octree_lines.attach(fs_octree_lines);
        sp_octree_lines.link();
        sp_octree_lines.use();

        U_projection = std::make_unique<Uniform<glm::mat4>>(sp_octree_lines.get_uniform<glm::mat4>("projection"));
        U_view = std::make_unique<Uniform<glm::mat4>>(sp_octree_lines.get_uniform<glm::mat4>("view"));

        glGenVertexArrays(1, &m_node_cube_vao);
        glBindVertexArray(m_node_cube_vao);

        LOG_DEBUG("CREATE CUBE_IBO");
        cube_ibo = std::make_unique<Buffer>(GL_ELEMENT_ARRAY_BUFFER, GL_STATIC_DRAW);
        LOG_DEBUG("CREATE CUBE_VBO");
        cube_vbo = std::make_unique<Buffer>();
        LOG_DEBUG("CREATE CUBE_INSTANCE_ACTIVE");
        cube_instance_active_buffer = std::make_unique<Buffer>();
        LOG_DEBUG("CREATE CUBE_INSTANCE_MODEL");
        cube_instance_model_buffer = std::make_unique<Buffer>();

        cube_vbo->set_data(UnitCube::vertices());
        cube_ibo->set_data(UnitCube::line_indices());
        m_cube_line_indices_size = UnitCube::line_indices().size();

        // VERTICES
        cube_vbo->bind();
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), NULL);

        // INSTANCE ACTIVE
        cube_instance_active_buffer->bind();
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(float), NULL);
        glVertexAttribDivisor(1, 1);

        // INSTANCE MODEL MATRICES
        cube_instance_model_buffer->bind();

        size_t vec4_size = sizeof(glm::vec4);

        // LOC 2: COLUMN 0
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 4 * vec4_size, (void *)0);
        glVertexAttribDivisor(2, 1);

        // LOC 3: COLUMN 1
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 4 * vec4_size, (void *)(1 * vec4_size));
        glVertexAttribDivisor(3, 1);

        // LOC 4: COLUMN 2
        glEnableVertexAttribArray(4);
        glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, 4 * vec4_size, (void *)(2 * vec4_size));
        glVertexAttribDivisor(4, 1);

        // LOC 5: COLUMN 3
        glEnableVertexAttribArray(5);
        glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, 4 * vec4_size, (void *)(3 * vec4_size));
        glVertexAttribDivisor(5, 1);

        // INDICES
        cube_ibo->bind();

        glBindVertexArray(0);
    }

    void OctreeRenderManager::init_node_mesh_rendering()
    {
        auto vsc_octree_mesh = RES.open("shaders/octree_mesh.vert");
        auto fsc_octree_mesh = RES.open("shaders/octree_mesh.frag");

        vs_octree_mesh.compile(std::string_view(vsc_octree_mesh.begin(), vsc_octree_mesh.end()));
        fs_octree_mesh.compile(std::string_view(fsc_octree_mesh.begin(), fsc_octree_mesh.end()));

        sp_octree_mesh.attach(vs_octree_mesh);
        sp_octree_mesh.attach(fs_octree_mesh);
        sp_octree_mesh.link();
        sp_octree_mesh.use();

        U_mesh_projection = std::make_unique<Uniform<glm::mat4>>(sp_octree_mesh.get_uniform<glm::mat4>("projection"));
        U_mesh_view = std::make_unique<Uniform<glm::mat4>>(sp_octree_mesh.get_uniform<glm::mat4>("view"));
        U_mesh_model = std::make_unique<Uniform<glm::mat4>>(sp_octree_mesh.get_uniform<glm::mat4>("model"));
        U_mesh_render_mode = std::make_unique<Uniform<float>>(sp_octree_mesh.get_uniform<float>("render_mode"));
        U_mesh_texture = std::make_unique<Uniform<int>>(sp_octree_mesh.get_uniform<int>("uTexture"));
        U_mesh_has_texture = std::make_unique<Uniform<bool>>(sp_octree_mesh.get_uniform<bool>("has_texture"));
    }
}