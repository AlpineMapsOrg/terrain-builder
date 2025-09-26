#include "OctreeWireframeRenderManager.h"
#include <core/geometry/RadixExtensions.h>
#include <core/geometry/UnitCube.h>
#include <log.h>

CMRC_DECLARE(res);

OctreeWireframeRenderManager::OctreeWireframeRenderManager(octree::Space space)
    : p_wireframe_ids(std::make_shared<SyncTrackedParam<std::vector<octree::Id>>>()),
      RES(cmrc::res::get_filesystem()),
      m_space(space),
      m_selection_opacity(0.1f),
      vs_wireframe(GL_VERTEX_SHADER), fs_wireframe(GL_FRAGMENT_SHADER),
      vs_selection(GL_VERTEX_SHADER)
{
    initialize();
}

unsigned int OctreeWireframeRenderManager::get_wireframe_node_count()
{
    if (!p_wireframe_ids->has_value())
    {
        return 0;
    }

    return p_wireframe_ids->value().size();
}

float OctreeWireframeRenderManager::get_selection_opacity()
{
    return m_selection_opacity;
}

void OctreeWireframeRenderManager::set_selection_opacity(float opacity)
{
    m_selection_opacity = opacity;
}

void OctreeWireframeRenderManager::set_selected_node(octree::Id id)
{
    LOG_DEBUG("Selected node: {}", id);
    m_selected_node = id;
}

std::optional<octree::Id> OctreeWireframeRenderManager::ray_cast_rendered_nodes(const std::shared_ptr<Camera> &camera)
{
    if (!p_wireframe_ids->has_value())
    {
        return std::nullopt;
    }

    glm::dvec3 ray_dir = camera->get_local_forward_dir();
    glm::dvec3 ray_start = camera->get_position();

    std::optional<octree::Id> closest_id;
    double closest_t = std::numeric_limits<double>::infinity();

    for (const octree::Id &id : p_wireframe_ids->value())
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

void OctreeWireframeRenderManager::update(const std::shared_ptr<Camera> &camera)
{
    if (!p_wireframe_ids->has_value())
    {
        return;
    }

    std::vector<glm::mat4>
        wireframe_cube_instance_models;

    for (auto &id : p_wireframe_ids->value())
    {
        auto bounds = m_space.get_node_bounds(id);

        glm::dmat4 model_scale = glm::scale(glm::dmat4(1.0f), bounds.size());
        glm::dmat4 model_translate = glm::translate(glm::dmat4(1.0f), bounds.centre() - camera->get_position());
        glm::mat4 model = model_translate * model_scale;

        wireframe_cube_instance_models.push_back(glm::mat4(model));
    }

    m_wireframe_cube_instances = wireframe_cube_instance_models.size();

    if (m_wireframe_cube_instances != 0)
    {
        m_wireframe_cube_instance_model_buffer->set_data(wireframe_cube_instance_models);
    }
}

void OctreeWireframeRenderManager::render(const std::shared_ptr<Camera> &camera)
{
    if (m_wireframe_cube_instances == 0)
    {
        return;
    }

    sp_wireframe.use();
    U_wireframe_view->set(camera->view_matrix());
    U_wireframe_projection->set(camera->projection_matrix());

    glBindVertexArray(m_wireframe_cube_vao);
    glDrawElementsInstanced(GL_LINES, m_wireframe_cube_indices_size, GL_UNSIGNED_INT, 0, m_wireframe_cube_instances);

    if (m_selected_node.has_value())
    {
        auto bounds = m_space.get_node_bounds(m_selected_node.value());

        glm::dmat4 model_scale = glm::scale(glm::dmat4(1.0f), bounds.size());
        glm::dmat4 model_translate = glm::translate(glm::dmat4(1.0f), bounds.centre() - camera->get_position());
        glm::mat4 model = model_translate * model_scale;

        sp_selection.use();
        U_selection_view->set(camera->view_matrix());
        U_selection_projection->set(camera->projection_matrix());
        U_selection_cube_model->set(model);
        U_selection_opacity->set(m_selection_opacity);

        glEnable(GL_CULL_FACE);

        glBindVertexArray(m_selection_cube_vao);
        glCullFace(GL_FRONT);
        glDrawElements(GL_TRIANGLES, m_selection_cube_indices_size, GL_UNSIGNED_INT, 0);
        glCullFace(GL_BACK);
        glDrawElements(GL_TRIANGLES, m_selection_cube_indices_size, GL_UNSIGNED_INT, 0);
        glDisable(GL_CULL_FACE);
    }
}

void OctreeWireframeRenderManager::initialize()
{
    auto vsc_selection = RES.open("shaders/selection.vert");
    vs_selection.compile(std::string_view(vsc_selection.begin(), vsc_selection.end()));

    auto vsc_wireframe = RES.open("shaders/wireframe.vert");
    vs_wireframe.compile(std::string_view(vsc_wireframe.begin(), vsc_wireframe.end()));

    auto fsc_wireframe = RES.open("shaders/wireframe.frag");
    fs_wireframe.compile(std::string_view(fsc_wireframe.begin(), fsc_wireframe.end()));

    sp_wireframe.attach(vs_wireframe);
    sp_wireframe.attach(fs_wireframe);
    sp_wireframe.link();

    sp_selection.attach(vs_selection);
    sp_selection.attach(fs_wireframe);
    sp_selection.link();

    U_wireframe_projection = std::make_unique<Uniform<glm::mat4>>(sp_wireframe.get_uniform<glm::mat4>("projection"));
    U_wireframe_view = std::make_unique<Uniform<glm::mat4>>(sp_wireframe.get_uniform<glm::mat4>("view"));

    U_selection_projection = std::make_unique<Uniform<glm::mat4>>(sp_selection.get_uniform<glm::mat4>("projection"));
    U_selection_view = std::make_unique<Uniform<glm::mat4>>(sp_selection.get_uniform<glm::mat4>("view"));
    U_selection_cube_model = std::make_unique<Uniform<glm::mat4>>(sp_selection.get_uniform<glm::mat4>("model"));
    U_selection_opacity = std::make_unique<Uniform<float>>(sp_selection.get_uniform<float>("opacity"));

    m_cube_vbo = std::make_unique<Buffer>();
    m_cube_vbo->set_data(UnitCube::vertices());

    // Selection Setup
    glGenVertexArrays(1, &m_selection_cube_vao);
    glBindVertexArray(m_selection_cube_vao);

    m_selection_cube_ibo = std::make_unique<Buffer>(GL_ELEMENT_ARRAY_BUFFER, GL_STATIC_DRAW);

    m_selection_cube_ibo->set_data(UnitCube::mesh_indices());
    m_selection_cube_indices_size = UnitCube::mesh_indices().size();

    // VERTICES
    m_cube_vbo->bind();
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), NULL);

    // INDICES
    m_selection_cube_ibo->bind();

    glBindVertexArray(0);
    // Selection Setup End

    // Wireframe Setup
    glGenVertexArrays(1, &m_wireframe_cube_vao);
    glBindVertexArray(m_wireframe_cube_vao);

    m_wireframe_cube_ibo = std::make_unique<Buffer>(GL_ELEMENT_ARRAY_BUFFER, GL_STATIC_DRAW);
    m_wireframe_cube_instance_model_buffer = std::make_unique<Buffer>();
    m_wireframe_cube_instances = 0;

    m_wireframe_cube_ibo->set_data(UnitCube::line_indices());
    m_wireframe_cube_indices_size = UnitCube::line_indices().size();

    // VERTICES
    m_cube_vbo->bind();
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), NULL);

    // INSTANCE MODEL MATRICES
    m_wireframe_cube_instance_model_buffer->bind();

    size_t vec4_size = sizeof(glm::vec4);

    // LOC 2: COLUMN 0
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 4 * vec4_size, (void *)0);
    glVertexAttribDivisor(1, 1);

    // LOC 3: COLUMN 1
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 4 * vec4_size, (void *)(1 * vec4_size));
    glVertexAttribDivisor(2, 1);

    // LOC 4: COLUMN 2
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 4 * vec4_size, (void *)(2 * vec4_size));
    glVertexAttribDivisor(3, 1);

    // LOC 5: COLUMN 3
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, 4 * vec4_size, (void *)(3 * vec4_size));
    glVertexAttribDivisor(4, 1);

    // INDICES
    m_wireframe_cube_ibo->bind();

    glBindVertexArray(0);
    // Wireframe Setup End
}