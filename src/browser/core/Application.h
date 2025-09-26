#pragma once
#include <glad/gl.h>

#include <GLFW/glfw3.h>

#include "Camera.h"
#include "io/OctreeNodeRepository.h"
#include "process/OctreeNodeCandidateSelector.h"
#include "rendering/OctreeMeshedRenderManager.h"
#include "rendering/OctreeWireframeRenderManager.h"
#include "shader/Uniform.h"
#include "window/Window.h"
#include <memory>
#include <octree/Storage.h>
#include <string>

class Application
{
public:
    Application(std::string title, int width, int height);

    void run(const std::vector<std::filesystem::path> &octree_indices);
    void update_camera(float frame_delta_time);

    ~Application();

private:
    std::string m_title;
    int m_width, m_height;

    std::unique_ptr<Window> m_window;
    std::shared_ptr<Camera> m_camera;

    octree::Space m_space;
    std::shared_ptr<OctreeNodeRepository> m_octree_repo;

    std::shared_ptr<OctreeWireframeRenderManager> m_octree_wireframe_render_manager;
    std::shared_ptr<OctreeMeshedRenderManager> m_octree_meshed_render_manager;
    std::unique_ptr<OctreeNodeCandidateSelector> m_octree_node_candidate_selector;

    float m_movement_speed, m_roll_speed, m_mouse_sensitivity;

    bool m_nav_mode;

    float m_refining_factor;

    size_t m_last_draw_amount;

    // Temporary user input variables
    // related to octree path inputs
    std::string m_tmp_new_path;
    bool m_tmp_path_is_file;
    bool m_tmp_path_is_valid;

    octree::Id::Level m_tmp_octree_zoom;
    octree::Id::Index m_tmp_octree_index;
    octree::Id::Coords m_tmp_octree_coords;
    std::optional<octree::Id> m_tmp_octree_id;

    // temporary node selector octree id input
    octree::Id::Level m_tmp_nsel_octree_zoom;
    octree::Id::Index m_tmp_nsel_octree_index;
    octree::Id::Coords m_tmp_nsel_octree_coords;
    std::optional<octree::Id> m_tmp_nsel_octree_id;
    bool m_tmp_nsel_octree_id_dirty;

    bool m_tmp_nsel_node_exists;
    std::optional<std::filesystem::path> m_tmp_nsel_node_path;
    std::optional<octree::NodeStatus> m_tmp_nsel_node_status;

    void toggle_nav_mode();

    void init_glad();
    void init_gl();

    void draw_settings_window();
    void draw_camera_settings_section();
    void draw_octree_settings_section();
    void draw_rendering_settings_section();

    static void gl_debug_callback(GLenum source, GLenum type, GLuint id,
                                  GLenum severity, GLsizei length,
                                  const GLchar *message,
                                  const GLvoid *userParam);
};