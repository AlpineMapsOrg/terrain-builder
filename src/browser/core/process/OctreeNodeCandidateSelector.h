#pragma once
#include "param/SyncTrackedParam.h"
#include <condition_variable>
#include <core/io/OctreeNodeRepository.h>
#include <core/threading/SafeQueue.h>
#include <glm/glm.hpp>
#include <mutex>
#include <set>
#include <thread>

class OctreeNodeCandidateSelector
{
public:
    std::shared_ptr<SyncTrackedParam<glm::dvec3>> p_cam_pos;
    std::shared_ptr<SyncTrackedParam<uint>> p_max_nodes;
    std::shared_ptr<SyncTrackedParam<uint>> p_max_meshed_nodes;
    std::shared_ptr<SyncTrackedParam<bool>> p_storage_changed;

    std::shared_ptr<SyncTrackedParam<std::vector<octree::Id>>> ep_wireframe_ids;
    std::shared_ptr<SafeQueue<std::tuple<octree::Id, std::shared_ptr<SimpleMesh>>>> ep_mesh_queue;
    std::shared_ptr<SafeQueue<octree::Id>> ep_removal_queue;

    OctreeNodeCandidateSelector(std::shared_ptr<SyncTrackedParam<std::vector<octree::Id>>> ep_wireframe_ids,
                                std::shared_ptr<SafeQueue<std::tuple<octree::Id, std::shared_ptr<SimpleMesh>>>> ep_mesh_queue,
                                std::shared_ptr<SafeQueue<octree::Id>> ep_removal_queue,
                                std::shared_ptr<OctreeNodeRepository> octree_node_repository, octree::Space space);
    ~OctreeNodeCandidateSelector();

    void start();
    void wake();

private:
    std::shared_ptr<OctreeNodeRepository> m_repository;
    octree::Space m_space;
    std::set<octree::Id> m_previous_meshed_ids;

    std::jthread m_thread;

    std::atomic_bool m_running;
    std::condition_variable m_wake_cv;

    void run();

    std::vector<octree::Id> calculate_candidates(glm::dvec3 cam_pos, uint max_nodes);
    void partition_meshed_candidates(const std::set<octree::Id> &candidates, std::vector<octree::Id> &removals, std::vector<octree::Id> &additions);
    void load_meshed_candidates(const std::vector<octree::Id> candidates);
};
