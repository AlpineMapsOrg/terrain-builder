#pragma once
#include <core/rendering/GPUOctreeNode.h>
#include <filesystem>
#include <mutex>
#include <octree/Storage.h>
#include <shared_mutex>
#include <unordered_map>

class OctreeNodeRepository
{
public:
    OctreeNodeRepository();

    bool register_index_folder(std::filesystem::path path_to_index_folder);
    bool register_file(std::filesystem::path path, octree::Id id);
    void unregister(const std::filesystem::path &path);

    std::vector<std::filesystem::path> get_registered_paths();
    std::vector<octree::Id> get_registered_file_ids();

    bool has_node(const octree::Id &id);
    std::optional<std::filesystem::path> get_node_file_path(const octree::Id &id);
    std::optional<octree::NodeStatus> get_node_status(const octree::Id &id);
    std::optional<std::shared_ptr<GPUOctreeNode>> load_node(const octree::Id &id, const octree::Space &space);
    std::optional<std::shared_ptr<SimpleMesh>> load_mesh(const octree::Id &id);
    std::vector<std::shared_ptr<GPUOctreeNode>> load_nodes(const std::vector<octree::Id> &ids, const octree::Space &space);

private:
    std::shared_mutex m_mutex;

    std::unordered_map<std::filesystem::path, std::unique_ptr<octree::Storage>> m_registered_storages;
    std::unordered_map<octree::Id, std::filesystem::path> m_registered_files;

    bool has_path_thorough_check(const std::filesystem::path &path_to_check);
};
