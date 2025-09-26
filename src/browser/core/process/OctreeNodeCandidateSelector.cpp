#include "OctreeNodeCandidateSelector.h"
#include <chrono>
#include <log.h>
#include <octree/Id.h>
#include <set>

using namespace std::chrono_literals;

OctreeNodeCandidateSelector::OctreeNodeCandidateSelector(std::shared_ptr<SyncTrackedParam<std::vector<octree::Id>>> ep_wireframe_ids,
                                                         std::shared_ptr<SafeQueue<std::tuple<octree::Id, std::shared_ptr<SimpleMesh>>>> ep_mesh_queue,
                                                         std::shared_ptr<SafeQueue<octree::Id>> ep_removal_queue,
                                                         std::shared_ptr<OctreeNodeRepository> octree_node_repository, octree::Space space)
    : p_cam_pos(std::make_shared<SyncTrackedParam<glm::dvec3>>()),
      p_max_nodes(std::make_shared<SyncTrackedParam<uint>>()),
      p_max_meshed_nodes(std::make_shared<SyncTrackedParam<uint>>()),
      p_storage_changed(std::make_shared<SyncTrackedParam<bool>>()),
      ep_wireframe_ids(ep_wireframe_ids),
      ep_mesh_queue(ep_mesh_queue),
      ep_removal_queue(ep_removal_queue),
      m_repository(octree_node_repository),
      m_space(space),
      m_thread(),
      m_running(false)
{
}

OctreeNodeCandidateSelector::~OctreeNodeCandidateSelector()
{
    LOG_DEBUG("[OctreeNodeCandidateSelector] Stopping");

    m_running = false;
    m_wake_cv.notify_one();
}

void OctreeNodeCandidateSelector::start()
{
    LOG_DEBUG("[OctreeNodeCandidateSelector] Starting");
    m_running = true;
    m_thread = std::jthread(&OctreeNodeCandidateSelector::run, this);
}

void OctreeNodeCandidateSelector::wake()
{
    m_wake_cv.notify_one();
}

void OctreeNodeCandidateSelector::run()
{
    std::mutex m;
    while (m_running)
    {
        std::unique_lock lock(m);
        m_wake_cv.wait(lock, [this]
                       { return p_cam_pos->has_changed() || p_max_nodes->has_changed() || p_max_meshed_nodes->has_changed() || p_storage_changed->has_changed() || !m_running; });

        if (!m_running)
        {
            break;
        }

        if (p_cam_pos->has_value() && p_max_nodes->has_value() && p_max_meshed_nodes->has_value() && p_storage_changed->has_value())
        {
            if (p_cam_pos->has_changed())
            {
                p_cam_pos->reset_changed();
            }
            if (p_max_nodes->has_changed())
            {
                p_max_nodes->reset_changed();
            }
            if (p_max_meshed_nodes->has_changed())
            {
                p_max_meshed_nodes->reset_changed();
            }
            if (p_storage_changed->has_changed())
            {
                p_storage_changed->write(false);
                p_storage_changed->reset_changed();
            }

            // LOG_DEBUG("[OctreeNodeCandidateSelector] Recalculating nodes with: p_cam_pos = {}, p_max_nodes = {}, p_max_meshed_nodes = {}", p_cam_pos->value(), p_max_nodes->value(), p_max_meshed_nodes->value());
            // auto start = std::chrono::high_resolution_clock::now();
            std::vector<octree::Id> candidates = calculate_candidates(p_cam_pos->value(), p_max_nodes->value());
            // auto finish = std::chrono::high_resolution_clock::now();

            // LOG_DEBUG("[OctreeNodeCandidateSelector] Got {} candidate_nodes in {}ms", candidates.size(), std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count());

            ep_wireframe_ids->write(candidates);

            candidates.resize(p_max_meshed_nodes->value());

            std::vector<octree::Id> additions;
            std::vector<octree::Id> removals;

            std::set<octree::Id> meshed_candidates_set(candidates.begin(), candidates.end());

            partition_meshed_candidates(meshed_candidates_set, removals, additions);

            m_previous_meshed_ids = meshed_candidates_set;

            for (octree::Id remove_id : removals)
            {
                ep_removal_queue->push(remove_id);
            }

            load_meshed_candidates(additions);
        }
    }
}

std::vector<octree::Id> OctreeNodeCandidateSelector::calculate_candidates(glm::dvec3 cam_pos, uint max_nodes)
{

    std::vector<octree::Id> candidate_octree_nodes;

    std::vector<octree::Id> id_traverse_queue;
    id_traverse_queue.push_back(octree::Id::root());

    while (!id_traverse_queue.empty() && candidate_octree_nodes.size() < max_nodes)
    {
        octree::Id current = id_traverse_queue.back();
        id_traverse_queue.pop_back();

        if (!m_repository->has_node(current))
        {
            // LOG_DEBUG("[OctreeNodeCandidateSelector] Skipping Node {}", current);
            continue;
        }

        auto status = m_repository->get_node_status(current);

        if (status.has_value() && status.value() == octree::NodeStatus::Virtual)
        {
            // LOG_DEBUG("[OctreeNodeCandidateSelector] Node {} is VIRTUAL splitting", current);

            std::array<octree::Id, 8> children = current.children().value();

            for (octree::Id child : children)
            {
                id_traverse_queue.push_back(child);
            }

            // Sort the whole queue based on the distance to the camera
            std::sort(id_traverse_queue.begin(), id_traverse_queue.end(), [this, cam_pos](octree::Id a, octree::Id b)
                      {
                            auto a_dist = glm::distance(m_space.get_node_bounds(a).centre(), cam_pos);
                            auto b_dist = glm::distance(m_space.get_node_bounds(b).centre(), cam_pos);

                            return a_dist > b_dist; });
        }
        else if (status.has_value() && status.value() == octree::NodeStatus::Inner)
        {
            // LOG_DEBUG("[OctreeNodeCandidateSelector] Node {} is INNER", current);
        }
        else if (status.has_value() && status.value() == octree::NodeStatus::Leaf)
        {
            // LOG_DEBUG("[OctreeNodeCandidateSelector] Node {} is LEAF", current);
            candidate_octree_nodes.push_back(current);
        }
    }

    // Add nodes from direct file sources
    auto registered_files = m_repository->get_registered_file_ids();
    candidate_octree_nodes.insert(candidate_octree_nodes.end(), registered_files.begin(), registered_files.end());

    // Remove duplicates
    std::set<octree::Id> s(candidate_octree_nodes.begin(), candidate_octree_nodes.end());
    candidate_octree_nodes.assign(s.begin(), s.end());

    // Sort the whole candidate list based on the distance to the camera
    std::sort(candidate_octree_nodes.begin(), candidate_octree_nodes.end(), [this, cam_pos](octree::Id a, octree::Id b)
              {
            auto a_dist = glm::distance(m_space.get_node_bounds(a).centre(), cam_pos);
            auto b_dist = glm::distance(m_space.get_node_bounds(b).centre(), cam_pos);
            
            return a_dist < b_dist; });

    if (candidate_octree_nodes.size() > max_nodes)
    {
        candidate_octree_nodes.resize(max_nodes);
    }

    return candidate_octree_nodes;
}

void OctreeNodeCandidateSelector::partition_meshed_candidates(const std::set<octree::Id> &candidates, std::vector<octree::Id> &removals, std::vector<octree::Id> &additions)
{
    for (octree::Id old_id : m_previous_meshed_ids)
    {
        if (!candidates.contains(old_id))
        {
            removals.push_back(old_id);
        }
    }

    for (octree::Id new_id : candidates)
    {
        if (!m_previous_meshed_ids.contains(new_id))
        {
            additions.push_back(new_id);
        }
    }
}

void OctreeNodeCandidateSelector::load_meshed_candidates(const std::vector<octree::Id> candidates)
{
    for (auto candidate : candidates)
    {
        auto mesh = m_repository->load_mesh(candidate);

        if (mesh.has_value())
        {
            ep_mesh_queue->push({candidate, mesh.value()});
        }
    }
}
