#include "OctreeNodeRepository.h"
#include <ranges>

OctreeNodeRepository::OctreeNodeRepository()
{
}

bool OctreeNodeRepository::register_index_folder(std::filesystem::path path_to_index_folder)
{
    path_to_index_folder = path_to_index_folder.lexically_normal();

    if (m_registered_storages.contains(path_to_index_folder))
    {
        LOG_WARN("Tried to register already registered octree index folder {}", path_to_index_folder.string());
        return false;
    }
    else if (has_path_thorough_check(path_to_index_folder))
    {
        LOG_WARN("Tried to register already registered octree index folder {}", path_to_index_folder.string());
        return false;
    }

    m_registered_storages[path_to_index_folder] = std::make_unique<octree::Storage>(octree::open_folder(path_to_index_folder));
    return true;
}

bool OctreeNodeRepository::register_file(std::filesystem::path path, octree::Id id)
{
    path = path.lexically_normal();

    if (m_registered_files.contains(id))
    {
        LOG_WARN("Tried to register already registered octree file {}", path.string());
        return false;
    }
    else if (has_path_thorough_check(path))
    {
        LOG_WARN("Tried to register already registered octree file {}", path.string());
        return false;
    }

    if (!std::filesystem::exists(path) || !std::filesystem::is_regular_file(path))
    {
        LOG_ERROR("Error when registering {}. File does not exist!", path);
        return false;
    }

    m_registered_files[id] = path;
    return true;
}

void OctreeNodeRepository::unregister(const std::filesystem::path &path)
{
    auto clean_path = path.lexically_normal();

    for (auto &entry : m_registered_files)
    {
        if (std::filesystem::equivalent(entry.second, clean_path))
        {
            m_registered_files.erase(entry.first);
            return;
        }
    }

    m_registered_storages.erase(clean_path);
}

std::vector<std::filesystem::path> OctreeNodeRepository::get_registered_paths()
{
    std::vector<std::filesystem::path> registered_paths;
    registered_paths.reserve(m_registered_storages.size() + m_registered_files.size());

    for (auto const &p : m_registered_files)
    {
        registered_paths.push_back(p.second);
    }

    for (auto const &p : m_registered_storages)
    {
        registered_paths.push_back(p.first);
    }

    return registered_paths;
}

std::vector<octree::Id> OctreeNodeRepository::get_registered_file_ids()
{
    std::vector<octree::Id> registered_file_ids;
    registered_file_ids.reserve(m_registered_files.size());

    for (auto const &p : m_registered_files)
    {
        registered_file_ids.push_back(p.first);
    }
    return registered_file_ids;
}

bool OctreeNodeRepository::has_node(const octree::Id &id)
{
    if (m_registered_files.contains(id))
    {
        return true;
    }

    for (auto &entry : m_registered_storages)
    {
        if (entry.second->has_node(id))
        {
            return true;
        }
    }

    return false;
}

std::optional<octree::NodeStatus> OctreeNodeRepository::get_node_status(const octree::Id &id)
{
    if (m_registered_files.contains(id))
    {
        // LOAD FROM FILE
        LOG_WARN("Can't determine node status of node {} directly from file {}. Try using an index instead.", id, m_registered_files[id]);
        return std::nullopt;
    }

    for (auto &entry : m_registered_storages)
    {
        if (entry.second->has_node(id))
        {
            return entry.second->index()->get(id);
        }
    }

    LOG_WARN("Tried to load node {} which does not exist in the registered locations", id);

    return std::nullopt;
}

std::optional<std::filesystem::path> OctreeNodeRepository::get_node_file_path(const octree::Id &id)
{
    if (m_registered_files.contains(id))
    {
        return m_registered_files[id].lexically_normal();
    }

    for (auto &entry : m_registered_storages)
    {
        if (entry.second->has_node(id))
        {
            return entry.second->get_node_path(id).lexically_normal();
        }
    }

    return std::nullopt;
}

std::optional<std::shared_ptr<GPUOctreeNode>> OctreeNodeRepository::load_node(const octree::Id &id, const octree::Space &space)
{
    std::optional<std::shared_ptr<GPUOctreeNode>> node = load_from_cache(id);

    if (node.has_value())
    {
        return node.value();
    }

    if (m_registered_files.contains(id))
    {
        // LOAD FROM FILE

        LOG_INFO("Loading {} from file {}", id, m_registered_files[id]);
        const auto node = mesh::io::load_from_path(m_registered_files[id]);
        if (!node.has_value())
        {
            LOG_ERROR("Failed loading {} from file {}", id, m_registered_files[id]);

            return std::nullopt;
        }

        std::shared_ptr<GPUOctreeNode> gpu_node = std::make_shared<GPUOctreeNode>(node.value(), id, space);

        store_in_cache(id, gpu_node);

        return gpu_node;
    }

    for (auto &entry : m_registered_storages)
    {
        if (entry.second->has_node(id))
        {
            // LOAD FROM STORAGE OBJECT
            LOG_INFO("Loading {} from storage {} at {}", id, entry.first, entry.second->get_node_path(id));

            auto node = entry.second->read_node(id);

            if (!node.has_value())
            {

                LOG_ERROR("Failed loading {} from storage {} at {}", id, entry.first, entry.second->get_node_path(id));

                return std::nullopt;
            }

            std::shared_ptr<GPUOctreeNode> gpu_node = std::make_shared<GPUOctreeNode>(node.value(), id, space);

            store_in_cache(id, gpu_node);

            return gpu_node;
        }
    }

    LOG_WARN("Tried to load node {} which does not exist in the registered locations", id);

    return std::nullopt;
}

std::vector<std::shared_ptr<GPUOctreeNode>> OctreeNodeRepository::load_nodes(const std::vector<octree::Id> &ids, const octree::Space &space)
{
    std::vector<std::shared_ptr<GPUOctreeNode>> nodes;

    for (octree::Id id : ids)
    {
        auto node = load_node(id, space);
        if (node.has_value())
        {
            nodes.push_back(node.value());
        }
    }

    return nodes;
}

std::optional<std::shared_ptr<GPUOctreeNode>> OctreeNodeRepository::load_from_cache(const octree::Id &id)
{
    if (m_node_cache.contains(id))
    {
        m_node_cache[id].second = std::chrono::system_clock::now();

        return m_node_cache[id].first;
    }
    return std::nullopt;
}

void OctreeNodeRepository::store_in_cache(const octree::Id &id, std::shared_ptr<GPUOctreeNode> node)
{
    if (m_node_cache.contains(id))
    {
        return;
    }

    // If the cache is full, free up some space
    if (m_node_cache.size() > m_max_cache_entries)
    {
        std::vector<std::pair<octree::Id, std::chrono::time_point<std::chrono::system_clock>>> cache_entries;

        for (auto &entry : m_node_cache)
        {
            cache_entries.push_back(std::make_pair(entry.first, entry.second.second));
        }

        std::sort(cache_entries.begin(), cache_entries.end(), [](std::pair<octree::Id, std::chrono::time_point<std::chrono::system_clock>> &a, std::pair<octree::Id, std::chrono::time_point<std::chrono::system_clock>> &b)
                  { return a.second < b.second; });

        while (m_node_cache.size() >= m_max_cache_entries)
        {
            m_node_cache.erase(cache_entries.front().first);
        }
    }
    m_node_cache[id] = std::make_pair(node, std::chrono::system_clock::now());
}

bool OctreeNodeRepository::has_path_thorough_check(const std::filesystem::path &path_to_check)
{
    for (auto &registered_file : m_registered_files)
    {
        if (std::filesystem::equivalent(registered_file.second, path_to_check))
        {
            return true;
        }
    }

    for (auto &registered_storage : m_registered_storages)
    {
        if (std::filesystem::equivalent(registered_storage.first, path_to_check))
        {
            return true;
        }
    }

    return false;
}
