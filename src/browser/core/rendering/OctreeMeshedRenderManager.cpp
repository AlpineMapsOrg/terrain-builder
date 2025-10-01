#include "OctreeMeshedRenderManager.h"

CMRC_DECLARE(res);

OctreeMeshedRenderManager::OctreeMeshedRenderManager(octree::Space space)
    : p_mesh_queue(std::make_shared<SafeQueue<std::tuple<octree::Id, std::shared_ptr<SimpleMesh>>>>()),
      p_removal_queue(std::make_shared<SafeQueue<octree::Id>>()),
      RES(cmrc::res::get_filesystem()),
      m_space(space),
      vs_mesh(GL_VERTEX_SHADER), fs_mesh(GL_FRAGMENT_SHADER)
{
    initialize();
    set_render_mode(RenderMode::Clay);
}

void OctreeMeshedRenderManager::set_render_mode(OctreeMeshedRenderManager::RenderMode new_render_mode)
{
    m_render_mode = new_render_mode;
}

OctreeMeshedRenderManager::RenderMode OctreeMeshedRenderManager::get_render_mode()
{
    return m_render_mode;
}

unsigned int OctreeMeshedRenderManager::get_meshed_node_count()
{
    return m_meshed_node_drawlist.size();
}

void OctreeMeshedRenderManager::update(const std::shared_ptr<Camera> &camera)
{

    auto start = std::chrono::high_resolution_clock::now();
    size_t del = 0;
    if (!p_removal_queue->empty())
    {
        auto id = p_removal_queue->pop();

        del += m_meshed_node_drawlist.erase(id);
    }
    if (del > 0)
    {
        auto finish = std::chrono::high_resolution_clock::now();
        LOG_DEBUG("Deleting {} elements took {}ms", del, std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count());
    }

    start = std::chrono::high_resolution_clock::now();
    size_t add = 0;
    if (!p_mesh_queue->empty())
    {
        auto [id, mesh] = p_mesh_queue->pop();

        // m_meshed_node_drawlist.erase(id);
        m_meshed_node_drawlist.emplace(id, std::make_unique<GPUOctreeNode>(mesh, id, m_space));
        add++;
    }
    if (add > 0)
    {
        auto finish = std::chrono::high_resolution_clock::now();
        LOG_DEBUG("Adding {} elements took {}ms", add, std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count());
    }

    glm::dvec3 cam_pos = camera->get_position();

    for (auto &[id, node] : m_meshed_node_drawlist)
    {
        node->recenter(cam_pos);
    }
}

void OctreeMeshedRenderManager::render(const std::shared_ptr<Camera> &camera)
{
    sp_mesh.use();

    apply_render_mode();

    U_mesh_view->set(camera->view_matrix());
    U_mesh_projection->set(camera->projection_matrix());

    sp_mesh.use();
    for (auto &[id, node] : m_meshed_node_drawlist)
    {
        U_mesh_model->set(node->model_matrix());
        node->render(U_mesh_texture, U_mesh_has_texture);
    }

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void OctreeMeshedRenderManager::initialize()
{
    auto vsc_mesh = RES.open("shaders/octree_mesh.vert");
    auto fsc_mesh = RES.open("shaders/octree_mesh.frag");

    vs_mesh.compile(std::string_view(vsc_mesh.begin(), vsc_mesh.end()));
    fs_mesh.compile(std::string_view(fsc_mesh.begin(), fsc_mesh.end()));

    sp_mesh.attach(vs_mesh);
    sp_mesh.attach(fs_mesh);
    sp_mesh.link();
    sp_mesh.use();

    U_mesh_projection = std::make_unique<Uniform<glm::mat4>>(sp_mesh.get_uniform<glm::mat4>("projection"));
    U_mesh_view = std::make_unique<Uniform<glm::mat4>>(sp_mesh.get_uniform<glm::mat4>("view"));
    U_mesh_model = std::make_unique<Uniform<glm::mat4>>(sp_mesh.get_uniform<glm::mat4>("model"));
    U_mesh_render_mode = std::make_unique<Uniform<float>>(sp_mesh.get_uniform<float>("render_mode"));
    U_mesh_texture = std::make_unique<Uniform<int>>(sp_mesh.get_uniform<int>("uTexture"));
    U_mesh_has_texture = std::make_unique<Uniform<bool>>(sp_mesh.get_uniform<bool>("has_texture"));
}

void OctreeMeshedRenderManager::apply_render_mode()
{
    glDisable(GL_CULL_FACE);

    if (m_render_mode == RenderMode::Wireframe)
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        U_mesh_render_mode->set(0);
    }
    else if (m_render_mode == RenderMode::Textured)
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        U_mesh_render_mode->set(1);
    }
    else if (m_render_mode == RenderMode::Textured_Unshaded)
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        U_mesh_render_mode->set(2);
    }
    else if (m_render_mode == RenderMode::Clay)
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        U_mesh_render_mode->set(3);
    }
    else if (m_render_mode == RenderMode::FlatNormals)
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        U_mesh_render_mode->set(4);
    }
    else if (m_render_mode == RenderMode::UVs)
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        U_mesh_render_mode->set(5);
    }
}
