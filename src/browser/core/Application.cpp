#include "Application.h"
#include <sstream>

#include <log.h>

#include <glm/glm.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <misc/cpp/imgui_stdlib.h>

#include "Buffer.h"
#include "Camera.h"
#include "geometry/UnitCube.h"
#include "octree/Space.h"
#include "rendering/GPUOctreeNode.h"
#include "rendering/ImGuiExtensions.h"
#include "shader/Shader.h"
#include "shader/ShaderProgram.h"

Application::Application(std::string title, int width, int height)
    : m_title(title), m_width(width), m_height(height), m_space(octree::Space::earth())
{

    m_nav_mode = false;

    m_refining_factor = 0.5f;

    WindowConfig c = {
        .width = 1280,
        .height = 720,
        .title = "Alpenite Browser",
        .resizeable = true,
        .msaa_samples = 4,
        .opengl_version = {4, 6},
        .opengl_core_profile = true};

    m_window = std::make_unique<Window>(c);

    init_glad();
    init_gl();

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(m_window->handle(), true); // Second param install_callback=true will install GLFW callbacks and chain to existing ones.
    ImGui_ImplOpenGL3_Init();

    if (c.msaa_samples > 0)
    {
        glEnable(GL_MULTISAMPLE);
    }

    m_movement_speed = 50.0f;
    m_roll_speed = 0.02f;
    m_mouse_sensitivity = 50.0f;

    m_tmp_path_is_file = false;
    m_tmp_path_is_valid = false;

    m_tmp_octree_id = octree::Id::root();
    m_tmp_octree_coords = m_tmp_octree_id.value().coords();
    m_tmp_octree_index = m_tmp_octree_id.value().index_on_level();
    m_tmp_octree_zoom = m_tmp_octree_id.value().level();

    m_tmp_nsel_octree_id = octree::Id::root();
    m_tmp_nsel_octree_id_dirty = true;
}

void Application::run(const std::vector<std::filesystem::path> &octree_indices)
{
    double last_frame_time = glfwGetTime();

    LOG_INFO("Setting up key event callbacks");
    m_window->register_key_event(GLFW_PRESS, GLFW_KEY_TAB, [this]()
                                 { toggle_nav_mode(); });

    m_window->register_key_event(GLFW_PRESS, GLFW_KEY_ESCAPE, [this]()
                                 {
                                     if (m_nav_mode)
                                     {
                                         toggle_nav_mode();
                                     }
                                     else
                                     {
                                         m_window->set_should_close(true);
                                     } });

    m_window->register_scroll_event([this](glm::dvec2 scroll)
                                    {
                                        if (!m_nav_mode)
                                        {
                                            return;
                                        }
                                        float old_speed = m_movement_speed;

                                        double factor = 1.0f;

                                        if (scroll.y > 0)
                                        {
                                            factor = 1.2f;
                                        }
                                        else if (scroll.y < 0)
                                        {
                                            factor = 0.8f;
                                        }

                                        m_movement_speed = glm::max(1.0f, m_movement_speed * (float)factor);
                                        LOG_DEBUG("Movement Speed: {} >> {}", old_speed, m_movement_speed); });

    m_window->register_mouse_button_event(GLFW_PRESS, GLFW_MOUSE_BUTTON_MIDDLE, [this]()
                                          {
                                              if (!m_nav_mode)
                                              {
                                                  return;
                                              }

                                              std::optional<octree::Id> picked_node = m_octree_render_manager->ray_cast_rendered_nodes(m_camera);

                                              if (picked_node.has_value())
                                              {
                                                  m_tmp_nsel_octree_id = picked_node;
                                                  m_tmp_nsel_octree_id_dirty = true;
                                                  m_octree_render_manager->set_selected_node(m_tmp_nsel_octree_id.value());
                                              } });

    LOG_INFO("Setting up camera");
    CameraConfig camera_config = {
        .fov_deg = 90.0f,
        .aspect_ratio = m_window->getAspectRatio(),
        .near_plane = 1.0f,
        .far_plane = 200000.0f,

        .position = glm::vec3(4081584.0f, 1203621.5f, 4735362.5f),
        .target = glm::vec3(0.0f),
        .up = glm::vec3(0.0f, 1.0f, 0.0f),
    };
    m_camera = std::make_shared<Camera>(camera_config);

    m_window->register_framebuffer_resize_event([this /*, &U_projection*/](glm::ivec2 new_size)
                                                {
                                                    if (new_size.y != 0) {
                                                        m_camera->set_aspect_ratio((float)new_size.x / (float)new_size.y);
                                                    }
                                                    
                                                    glViewport(0, 0, new_size.x, new_size.y);
                                                    
                                                    m_octree_render_manager->U_projection->set(m_camera->projection_matrix()); });

    LOG_INFO("Setting up Octree repository");
    m_octree_repo = std::make_shared<OctreeNodeRepository>();

    for (auto &index : octree_indices)
    {
        m_octree_repo->register_index_folder(index);
    }

    LOG_INFO("Setting up OctreeRenderManager");

    m_octree_render_manager = std::make_shared<octree::OctreeRenderManager>(m_octree_repo, m_space);

    glViewport(0, 0, m_window->get_window_size().x, m_window->get_window_size().y);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glClearColor(0.f, 0.f, 0.f, 1.f);

    glfwSwapInterval(1);

    m_last_draw_amount = 0;

    LOG_DEBUG("START LOOP");

    while (!m_window->should_close())
    {

        m_window->poll_events();

        const double current_frame_time = glfwGetTime();
        const double frame_delta_time = current_frame_time - last_frame_time;
        last_frame_time = current_frame_time;

        update_camera(frame_delta_time);

        m_octree_render_manager->update(m_camera);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        m_octree_render_manager->render();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        draw_settings_window();

        // RENDER IMGUI AFTER OUR RENDERS
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        m_window->swapBuffers();
    }
}

void Application::update_camera(float frame_delta_time)
{
    if (!m_camera)
    {
        LOG_WARN("Trying to update non-initialized camera!");
        return;
    }

    if (!m_nav_mode)
    {
        return;
    }

    /*glm::dvec2 scroll_delta = m_window->get_accumulated_scroll_delta();
    m_movement_speed = glm::max(m_movement_speed + glm::sign((float)scroll_delta.y), 0.0f);*/

    float movement_speed = m_movement_speed;
    float roll_speed = m_roll_speed;
    float mouse_sensitivity = m_mouse_sensitivity;

    if (m_window->is_key_pressed(GLFW_KEY_LEFT_SHIFT))
    {
        movement_speed *= 5.0f;
        roll_speed *= 1.5f;
    }

    float roll = 0.0f;
    if (m_window->is_key_pressed(GLFW_KEY_Q))
    {
        roll -= 1.00f;
    }
    if (m_window->is_key_pressed(GLFW_KEY_E))
    {
        roll += 1.00f;
    }

    const glm::dvec2 cursor_position_delta = m_window->get_accumulated_cursor_delta();
    glm::vec3 camera_rotation_delta = glm::vec3(-cursor_position_delta.x, -cursor_position_delta.y, roll * roll_speed) * (float)frame_delta_time * mouse_sensitivity;

    if (glm::abs(camera_rotation_delta.x) + glm::abs(camera_rotation_delta.y) + glm::abs(camera_rotation_delta.z) != 0.0f)
    {
        m_camera->rotate(camera_rotation_delta.x, camera_rotation_delta.y, camera_rotation_delta.z);
    }

    glm::vec3 local_movement_delta(0.0f);

    if (m_window->is_key_pressed(GLFW_KEY_W) || m_window->is_key_pressed(GLFW_KEY_UP))
    {
        local_movement_delta.z += 1.0f;
    }
    if (m_window->is_key_pressed(GLFW_KEY_S) || m_window->is_key_pressed(GLFW_KEY_DOWN))
    {
        local_movement_delta.z -= 1.0f;
    }

    if (m_window->is_key_pressed(GLFW_KEY_D) || m_window->is_key_pressed(GLFW_KEY_RIGHT))
    {
        local_movement_delta.x += 1.0f;
    }
    if (m_window->is_key_pressed(GLFW_KEY_A) || m_window->is_key_pressed(GLFW_KEY_LEFT))
    {
        local_movement_delta.x -= 1.0f;
    }

    if (m_window->is_key_pressed(GLFW_KEY_SPACE))
    {
        local_movement_delta.y += 1.0f;
    }
    if (m_window->is_key_pressed(GLFW_KEY_LEFT_CONTROL))
    {
        local_movement_delta.y -= 1.0f;
    }

    float movement_magnitude = glm::length(local_movement_delta);
    local_movement_delta /= movement_magnitude;

    if (movement_magnitude != 0.0f)
    {
        m_camera->move_local(local_movement_delta * (float)frame_delta_time * movement_speed);
    }
}

Application::~Application()
{
    LOG_INFO("Exiting");
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void Application::toggle_nav_mode()
{
    m_nav_mode = !m_nav_mode;

    ImGuiIO &io = ImGui::GetIO();

    if (m_nav_mode && io.WantCaptureKeyboard)
    {
        // When Dear ImGui wants to capture the keyboard, do not switch to nav mode
        m_nav_mode = false;
    }

    if (m_nav_mode)
    {
        LOG_DEBUG("Entering Nav Mode");

        io.ConfigFlags |= ImGuiConfigFlags_NoMouse;
        io.ConfigFlags |= ImGuiConfigFlags_NoKeyboard;

        m_window->set_capture_mouse(true);
        m_window->clear_accumulated_cursor_delta();
    }
    else
    {
        LOG_DEBUG("Exiting Nav Mode");

        io.ConfigFlags &= ~ImGuiConfigFlags_NoMouse;
        io.ConfigFlags &= ~ImGuiConfigFlags_NoKeyboard;

        m_window->set_capture_mouse(false);
    }
}

void Application::init_glad()
{
    // Load OpenGL functions, gladLoadGL returns the loaded version, 0 on error.
    int version = gladLoadGL(glfwGetProcAddress);
    if (version == 0)
    {
        LOG_ERROR_AND_EXIT("Failed to initialize OpenGL context");
    }

    // Successfully loaded OpenGL
    LOG_INFO("Loaded OpenGL {0}.{1}", GLAD_VERSION_MAJOR(version), GLAD_VERSION_MINOR(version));
}

void Application::init_gl()
{
#ifdef _DEBUG
    glEnable(GL_DEBUG_OUTPUT);
    // set debug callback
    glDebugMessageCallback(gl_debug_callback, nullptr);
#endif // _DEBUG

    glViewport(0, 0, m_width, m_height);
}

void Application::draw_settings_window()
{
    const ImGuiViewport *main_viewport = ImGui::GetMainViewport();

    float window_width = glm::clamp(main_viewport->Size.x * 0.2f, 250.0f, 350.0f);
    float window_height = main_viewport->Size.y;

    ImGui::SetNextWindowSizeConstraints(
        ImVec2(window_width, window_height),         // min size
        ImVec2(main_viewport->Size.x, window_height) // max size
    );
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(window_width, window_height), ImGuiCond_Appearing);

    ImGuiWindowFlags flags = 0;
    flags |= ImGuiWindowFlags_NoMove;
    // flags |= ImGuiWindowFlags_NoResize;

    if (!ImGui::Begin("Settings", NULL, flags))
    {
        ImGui::End();
        return;
    }

    draw_rendering_settings_section();
    draw_octree_settings_section();
    draw_camera_settings_section();

    ImGui::End();
}

void Application::draw_camera_settings_section()
{
    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (!ImGui::CollapsingHeader("Camera"))
    {
        return;
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode("Position"))
    {
        bool cam_pos_edited = false;
        glm::dvec3 cam_pos = m_camera->get_position();

        ImGui::PushItemWidth(-80);

        ImGui::InputDouble("X", &cam_pos.x);

        if (ImGui::IsItemEdited())
        {
            cam_pos_edited = true;
        }

        // ImGui::SetNextItemWidth(double_input_width);
        ImGui::InputDouble("Y", &cam_pos.y);

        if (ImGui::IsItemEdited())
        {
            cam_pos_edited = true;
        }

        // ImGui::SetNextItemWidth(double_input_width);
        ImGui::InputDouble("Z", &cam_pos.z);

        if (ImGui::IsItemEdited() || cam_pos_edited)
        {
            cam_pos_edited = true;

            m_camera->set_position(cam_pos);
        }

        ImGui::TreePop();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode("Orientation"))
    {
        ImGui::SeparatorText("Euler Angles");
        bool euler_edited = false;
        glm::vec3 euler = glm::degrees(m_camera->get_rotation_euler());

        ImGui::SliderFloat("Pitch (X)", &euler.x, -180.0f, 180.0f);
        if (ImGui::IsItemEdited())
        {
            euler_edited = true;
        }

        ImGui::SliderFloat("Yaw (Y)", &euler.y, -90.0f, 90.0f);
        if (ImGui::IsItemEdited())
        {
            euler_edited = true;
        }

        ImGui::SliderFloat("Roll (Z)", &euler.z, -180.0f, 180.0f);
        if (ImGui::IsItemEdited() || euler_edited)
        {
            euler_edited = true;

            m_camera->set_rotation_euler(glm::radians(euler));
        }

        ImGui::SeparatorText("Quaternion");
        bool quat_edited = false;
        glm::quat quat = m_camera->get_rotation_quat();

        ImGui::SliderFloat("X", &quat.x, -1.0, 1.0);
        if (ImGui::IsItemEdited())
        {
            quat_edited = true;
        }

        ImGui::SliderFloat("Y", &quat.y, -1.0, 1.0);
        if (ImGui::IsItemEdited())
        {
            quat_edited = true;
        }

        ImGui::SliderFloat("Z", &quat.z, -1.0, 1.0);
        if (ImGui::IsItemEdited())
        {
            quat_edited = true;
        }

        ImGui::SliderFloat("W", &quat.w, -1.0, 1.0);
        if (ImGui::IsItemEdited() || quat_edited)
        {
            quat_edited = true;

            m_camera->set_rotation_quat(glm::normalize(quat));
        }

        ImGui::TreePop();
    }

    // ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode("Projection"))
    {
        float fov = m_camera->get_fov();

        ImGui::SliderFloat("FOV", &fov, 0.1f, 120.0f);
        if (ImGui::IsItemEdited())
        {
            m_camera->set_fov(fov);
        }

        ImGui::Separator();

        float near = m_camera->get_near();
        ImGui::InputFloat("Near Plane", &near);
        if (ImGui::IsItemEdited())
        {
            m_camera->set_near(near);
        }

        float far = m_camera->get_far();
        ImGui::InputFloat("Far Plane", &far);
        if (ImGui::IsItemEdited())
        {
            m_camera->set_far(far);
        }

        ImGui::Separator();

        ImGui::TreePop();
    }
}

void Application::draw_octree_settings_section()
{
    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (!ImGui::CollapsingHeader("Octree"))
    {
        return;
    }

    ImGui::PushItemWidth(-80);

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode("Node Selector"))
    {
        if (m_tmp_nsel_octree_id_dirty && m_tmp_nsel_octree_id.has_value())
        {
            m_tmp_nsel_octree_zoom = m_tmp_nsel_octree_id.value().level();
            m_tmp_nsel_octree_coords = m_tmp_nsel_octree_id.value().coords();
            m_tmp_nsel_octree_index = m_tmp_nsel_octree_id.value().index_on_level();

            m_tmp_nsel_node_path = m_octree_repo->get_node_file_path(m_tmp_nsel_octree_id.value());
            m_tmp_nsel_node_status = m_octree_repo->get_node_status(m_tmp_nsel_octree_id.value());
            m_tmp_nsel_node_exists = m_octree_repo->has_node(m_tmp_nsel_octree_id.value());

            m_tmp_nsel_octree_id_dirty = false;
        }

        if (ImGui::ALP::InputOctreeId(m_tmp_nsel_octree_id, m_tmp_nsel_octree_zoom, m_tmp_nsel_octree_coords, m_tmp_nsel_octree_index))
        {
            m_tmp_nsel_octree_id_dirty = true;

            if (m_tmp_nsel_octree_id.has_value())
            {
                m_octree_render_manager->set_selected_node(m_tmp_nsel_octree_id.value());
            }
        }

        ImGui::SeparatorText("Node Information");

        static ImGuiTableFlags flags;
        flags |= ImGuiTableFlags_SizingFixedFit;
        flags |= ImGuiTableFlags_Resizable;
        flags |= ImGuiTableFlags_BordersOuter;
        flags |= ImGuiTableFlags_BordersV;
        flags |= ImGuiTableFlags_ContextMenuInBody;
        flags |= ImGuiTableFlags_ScrollX | ImGuiTableFlags_ScrollY;

        ImVec2 table_viewport_size(0.0f, 100.0f);
        if (ImGui::BeginTable("node_info", 2, flags, table_viewport_size, 1000.0f))
        {
            ImGui::TableSetupScrollFreeze(1, 1);
            ImGui::TableSetupColumn("Property", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();

            ImGui::TableNextRow();

            ImGui::TableSetColumnIndex(0);
            ImGui::Text("Exists");

            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%s", m_tmp_nsel_node_exists ? "Yes" : "No");

            ImGui::TableNextRow();

            ImGui::TableSetColumnIndex(0);
            ImGui::Text("NodeStatus");

            ImGui::TableSetColumnIndex(1);
            std::string status = "Unknown";

            if (m_tmp_nsel_node_status.has_value())
            {
                switch (m_tmp_nsel_node_status.value())
                {
                case octree::NodeStatus::Inner:
                    status = "Inner";
                    break;
                case octree::NodeStatus::Virtual:
                    status = "Virtual";
                    break;
                case octree::NodeStatus::Leaf:
                    status = "Leaf";
                    break;
                }
            }
            else if (m_tmp_nsel_node_path.has_value())
            {
                // Node exists, has a path, but the status is unknown, so its probably coming from a directly added file
                status = "Unknown due to non-indexed file";
            }

            ImGui::Text("%s", status.c_str());

            ImGui::TableNextRow();

            ImGui::TableSetColumnIndex(0);
            ImGui::Text("FilePath");

            ImGui::TableSetColumnIndex(1);
            std::string path = "Unknown";

            if (m_tmp_nsel_node_path.has_value())
            {
                path = m_tmp_nsel_node_path.value().string();
            }

            ImGui::Text("%s", path.c_str());

            ImGui::EndTable();
        }

        ImGui::SeparatorText("Actions");

        ImGui::BeginDisabled(!m_tmp_nsel_octree_id.has_value());

        if (ImGui::Button("Jump To") && m_tmp_nsel_octree_id.has_value())
        {
            octree::Bounds node_bounds = m_space.get_node_bounds(m_tmp_nsel_octree_id.value());
            glm::dvec3 node_centre = node_bounds.centre();

            glm::dvec3 node_to_cam_dir = m_camera->get_local_forward_dir() * -glm::length(node_bounds.size());

            m_camera->set_position(node_centre + node_to_cam_dir);
        }

        ImGui::EndDisabled();

        ImGui::TreePop();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode("Input"))
    {
        float path_input_border_size = m_tmp_path_is_valid || m_tmp_new_path.size() <= 0 ? 0.0f : 1.0f;

        ImGui::PushStyleColor(ImGuiCol_Border, (ImVec4)ImColor::HSV(0, 1, 1));
        ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, path_input_border_size);
        if (ImGui::InputText("Path", &m_tmp_new_path))
        {
            if (std::filesystem::exists(m_tmp_new_path))
            {
                if (std::filesystem::is_directory(m_tmp_new_path))
                {
                    m_tmp_path_is_file = false;
                    m_tmp_path_is_valid = true;
                }
                else if (std::filesystem::is_regular_file(m_tmp_new_path))
                {
                    m_tmp_path_is_file = true;
                    m_tmp_path_is_valid = true;
                }
            }
            else
            {
                m_tmp_path_is_file = false;
                m_tmp_path_is_valid = false;
            }
        }

        ImGui::PopStyleColor();
        ImGui::PopStyleVar();

        ImGui::Separator();

        // Disable Octree ID Input when the path given is a directory
        ImGui::BeginDisabled(!m_tmp_path_is_file || !m_tmp_path_is_valid);
        ImGui::ALP::InputOctreeId(m_tmp_octree_id, m_tmp_octree_zoom, m_tmp_octree_coords, m_tmp_octree_index);
        ImGui::EndDisabled();

        ImGui::Separator();

        // Disable "Add" Button if the path empty or if the octree id is invalid if the input is a file
        ImGui::BeginDisabled(!m_tmp_path_is_valid || (m_tmp_path_is_file && !m_tmp_octree_id.has_value()));

        if (ImGui::Button("Add"))
        {
            if (m_tmp_path_is_file && m_octree_repo->register_file(m_tmp_new_path, m_tmp_octree_id.value()))
            {
                m_tmp_new_path.clear();
                m_tmp_octree_id = octree::Id::root();

                m_tmp_octree_coords = m_tmp_octree_id.value().coords();
                m_tmp_octree_index = m_tmp_octree_id.value().index_on_level();
                m_tmp_octree_zoom = m_tmp_octree_id.value().level();
            }
            else if (!m_tmp_path_is_file && m_octree_repo->register_index_folder(m_tmp_new_path))
            {
                m_tmp_new_path.clear();
            }
        }

        ImGui::EndDisabled();

        ImGui::SeparatorText("Registered Locations");

        static ImGuiTableFlags flags;
        flags |= ImGuiTableFlags_SizingFixedFit;
        flags |= ImGuiTableFlags_Resizable;
        flags |= ImGuiTableFlags_BordersOuter;
        flags |= ImGuiTableFlags_BordersV;
        flags |= ImGuiTableFlags_ContextMenuInBody;
        flags |= ImGuiTableFlags_ScrollX | ImGuiTableFlags_ScrollY;

        ImVec2 table_viewport_size(0.0f, 100.0f);
        if (ImGui::BeginTable("octree_node_repo", 2, flags, table_viewport_size, 1000.0f))
        {
            ImGui::TableSetupScrollFreeze(1, 1);
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("Path", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();

            int i = 0;
            for (auto path : m_octree_repo->get_registered_paths())
            {
                ImGui::PushID(i);
                ImGui::TableNextRow();

                ImGui::TableSetColumnIndex(0);
                if (ImGui::Button("Del"))
                {
                    m_octree_repo->unregister(path);
                }

                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%s", path.string().c_str());

                ImGui::PopID();
                i++;
            }
            ImGui::EndTable();
        }

        // ImGui::Text("Nodes found on disk: %d", -1);

        ImGui::TreePop();
    }

    // ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode("Stats"))
    {

        ImGui::Text("Nodes rendered: %d", m_octree_render_manager->get_last_node_draw_amount());

        ImGui::TreePop();
    }

    // ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    // if (ImGui::TreeNode("Filtering"))
    // {

    //     ImGui::TreePop();
    // }

    // ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    // if (ImGui::TreeNode("Refining"))
    // {
    //     std::array<std::string, 3> metrics = {"Distance", "Level", "DGNSDNFOL"};
    //     size_t selected_idx = 0;

    //     if (ImGui::BeginCombo("Metric", metrics[selected_idx].c_str()))
    //     {
    //         for (int i = 0; i < metrics.size(); i++)
    //         {
    //             bool selected = selected_idx == i;

    //             if (ImGui::Selectable(metrics[i].c_str(), selected))
    //             {
    //                 selected_idx = i;
    //                 selected = true;
    //             }

    //             if (selected)
    //             {
    //                 ImGui::SetItemDefaultFocus();
    //             }
    //         }
    //         ImGui::EndCombo();
    //     }

    //     if (ImGui::SliderFloat("Factor", &m_refining_factor, 0.0f, 2.0f))
    //     {
    //     }

    //     ImGui::TreePop();
    // }

    ImGui::PopItemWidth();
}

void Application::draw_rendering_settings_section()
{
    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (!ImGui::CollapsingHeader("Render Settings"))
    {
        return;
    }

    std::array<std::string, 5> modes = {"Wireframe", "Textured", "Clay", "FlatNormals", "UVs"};
    size_t selected_idx = 0;

    switch (m_octree_render_manager->get_render_mode())
    {
    case octree::RenderMode::Wireframe:
        selected_idx = 0;
        break;
    case octree::RenderMode::Textured:
        selected_idx = 1;
        break;
    case octree::RenderMode::Clay:
        selected_idx = 2;
        break;
    case octree::RenderMode::FlatNormals:
        selected_idx = 3;
        break;
    case octree::RenderMode::UVs:
        selected_idx = 4;
        break;
    }

    if (ImGui::BeginCombo("Render Mode", modes[selected_idx].c_str()))
    {
        for (size_t i = 0; i < modes.size(); i++)
        {
            bool selected = selected_idx == i;

            if (ImGui::Selectable(modes[i].c_str(), selected))
            {
                selected_idx = i;
                selected = true;
            }

            if (selected)
            {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();

        switch (selected_idx)
        {
        case 0:
            m_octree_render_manager->set_render_mode(octree::RenderMode::Wireframe);
            break;
        case 1:
            m_octree_render_manager->set_render_mode(octree::RenderMode::Textured);
            break;
        case 2:
            m_octree_render_manager->set_render_mode(octree::RenderMode::Clay);
            break;
        case 3:
            m_octree_render_manager->set_render_mode(octree::RenderMode::FlatNormals);
            break;
        case 4:
            m_octree_render_manager->set_render_mode(octree::RenderMode::UVs);
            break;
        }
    }
}

void Application::gl_debug_callback(GLenum source, GLenum type,
                                    GLuint id, GLenum severity,
                                    GLsizei length,
                                    const GLchar *message,
                                    const GLvoid *userParam)
{
    std::stringstream stringStream;
    std::string sourceString;
    std::string typeString;
    std::string severityString;

    switch (source)
    {
    case GL_DEBUG_SOURCE_API:
    {
        sourceString = "API";
        break;
    }
    case GL_DEBUG_SOURCE_APPLICATION:
    {
        sourceString = "Application";
        break;
    }
    case GL_DEBUG_SOURCE_WINDOW_SYSTEM:
    {
        sourceString = "Window System";
        break;
    }
    case GL_DEBUG_SOURCE_SHADER_COMPILER:
    {
        sourceString = "Shader Compiler";
        break;
    }
    case GL_DEBUG_SOURCE_THIRD_PARTY:
    {
        sourceString = "Third Party";
        break;
    }
    case GL_DEBUG_SOURCE_OTHER:
    {
        sourceString = "Other";
        break;
    }
    default:
    {
        sourceString = "Unknown";
        break;
    }
    }

    switch (type)
    {
    case GL_DEBUG_TYPE_ERROR:
    {
        typeString = "Error";
        break;
    }
    case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
    {
        typeString = "Deprecated Behavior";
        break;
    }
    case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
    {
        typeString = "Undefined Behavior";
        break;
    }
    case GL_DEBUG_TYPE_PORTABILITY_ARB:
    {
        typeString = "Portability";
        break;
    }
    case GL_DEBUG_TYPE_PERFORMANCE:
    {
        typeString = "Performance";
        break;
    }
    case GL_DEBUG_TYPE_OTHER:
    {
        typeString = "Other";
        break;
    }
    default:
    {
        typeString = "Unknown";
        break;
    }
    }

    switch (severity)
    {
    case GL_DEBUG_SEVERITY_HIGH:
    {
        severityString = "High";
        break;
    }
    case GL_DEBUG_SEVERITY_MEDIUM:
    {
        severityString = "Medium";
        break;
    }
    case GL_DEBUG_SEVERITY_LOW:
    {
        severityString = "Low";
        break;
    }
    case GL_DEBUG_SEVERITY_NOTIFICATION:
    {
        severityString = "Notification";
        break;
    }
    default:
    {
        severityString = "Unknown";
        break;
    }
    }

    stringStream << message;
    stringStream << " [Source = " << sourceString;
    stringStream << ", Type = " << typeString;
    stringStream << ", Severity = " << severityString;
    stringStream << ", ID = " << id << "]";

    switch (severity)
    {
    case GL_DEBUG_SEVERITY_HIGH:
    {
        LOG_ERROR(stringStream.str());
        break;
    }
    case GL_DEBUG_SEVERITY_MEDIUM:
    {
        LOG_WARN(stringStream.str());
        break;
    }
    case GL_DEBUG_SEVERITY_LOW:
    {
        LOG_INFO(stringStream.str());
        break;
    }
    case GL_DEBUG_SEVERITY_NOTIFICATION:
    {
        LOG_DEBUG(stringStream.str());
        break;
    }
    default:
    {
        LOG_TRACE(stringStream.str());
        break;
    }
    }
}
