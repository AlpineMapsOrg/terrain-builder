#pragma once
#include <core/rendering/GPUOctreeNode.h>
#include <filesystem>
#include <octree/Storage.h>
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

    uint get_max_cache_entries();
    void set_max_cache_entries(uint new_max_cache_entries);

    bool has_node(const octree::Id &id);
    std::optional<std::filesystem::path> get_node_file_path(const octree::Id &id);
    std::optional<octree::NodeStatus> get_node_status(const octree::Id &id);
    std::optional<std::shared_ptr<GPUOctreeNode>> load_node(const octree::Id &id, const octree::Space &space);
    std::vector<std::shared_ptr<GPUOctreeNode>> load_nodes(const std::vector<octree::Id> &ids, const octree::Space &space);

private:
    std::unordered_map<std::filesystem::path, std::unique_ptr<octree::Storage>> m_registered_storages;
    std::unordered_map<octree::Id, std::filesystem::path> m_registered_files;

    std::unordered_map<octree::Id, std::pair<std::shared_ptr<GPUOctreeNode>, std::chrono::time_point<std::chrono::system_clock>>> m_node_cache;
    uint m_max_cache_entries = 50;

    std::optional<std::shared_ptr<GPUOctreeNode>> load_from_cache(const octree::Id &id);
    void store_in_cache(const octree::Id &id, std::shared_ptr<GPUOctreeNode> node);

    bool has_path_thorough_check(const std::filesystem::path &path_to_check);
};
