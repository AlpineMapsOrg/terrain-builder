/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#pragma once

#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <utility>

#include <expected>

#include "cli_support/log.h"
#include "store/IndexFormat.h"
#include "store/RawStorage.h"

namespace store {

template <HierarchyTraits Traits, typename NodeData>
class Storage {
public:
    using Key = typename Traits::Key;
    using key_type = Key;
    using value_type = NodeData;

    struct Persistence {
        IndexFormat<Traits> format;
        std::filesystem::path index_path;
        std::string layout_id;
        std::string payload_class;
        std::string codec_selector;
    };

    Storage(RawStorage<Traits, NodeData> raw, Index<Traits> index, Persistence persistence, const bool dirty = false)
        : m_raw(std::move(raw))
        , m_index(std::move(index))
        , m_persistence(std::move(persistence))
        , m_dirty(dirty)
    {
    }

    virtual ~Storage() { finalize_displaced_state(); }

    Storage(const Storage&) = delete;
    Storage& operator=(const Storage&) = delete;

    Storage(Storage&& other) noexcept
        : m_raw(std::move(other.m_raw))
        , m_index(std::move(other.m_index))
        , m_persistence(std::move(other.m_persistence))
        , m_dirty(std::exchange(other.m_dirty, false))
    {
        other.m_index.reset();
        other.m_persistence.reset();
    }

    Storage& operator=(Storage&& other) noexcept
    {
        if (this == &other) {
            return *this;
        }
        finalize_displaced_state();
        m_raw = std::move(other.m_raw);
        m_index = std::move(other.m_index);
        m_persistence = std::move(other.m_persistence);
        m_dirty = std::exchange(other.m_dirty, false);
        other.m_index.reset();
        other.m_persistence.reset();
        return *this;
    }

    Expected<NodeData> load(const Key& key) const
    {
        if (!Traits::is_valid(key)) {
            return store::invalid_key_error<Traits>(key);
        }
        auto status = m_index->get(key);
        if (!status) {
            return Error::propagate(std::move(status), "look up node " + Traits::key_to_string(key) + " in storage index before loading");
        }
        if (!status->has_value() || status->value() == NodeStatus::Virtual) {
            return Error::fail(Error::Code::NotFound, "node " + Traits::key_to_string(key) + " is not physically present in the index");
        }
        return m_raw.load(key);
    }

    Expected<void> save(const Key& key, const NodeData& data)
    {
        if (!Traits::is_valid(key)) {
            return store::invalid_key_error<Traits>(key);
        }
        auto exists = has(key);
        if (!exists) {
            return Error::propagate(std::move(exists), "check whether node " + Traits::key_to_string(key) + " exists before saving");
        }
        if (exists.value()) {
            const auto node_paths = m_raw.paths(key).value();
            return Error::fail(Error::Code::AlreadyExists, "save node to", node_paths.empty() ? m_raw.layout().node_path(key) : node_paths.front());
        }

        auto result = write_payload(key, data);
        if (!result) {
            return result;
        }
        return commit(key);
    }

    // Writes the node's payload files without recording the node in the index;
    // commit records it. save is the existence check, write_payload and commit.
    // Thread safety: may run concurrently with itself and with any other method
    // except construction, move, assignment and destruction. It reads only the
    // layout and the codec, which never change after construction, and never the
    // index. Codecs create files exclusively, so a second write of a key, concurrent
    // or later, fails with AlreadyExists and leaves the first write's files intact.
    // Payload files must not be removed or replaced externally while writes are
    // outstanding. Linking from this storage requires completed, immutable payloads.
    Expected<void> write_payload(const Key& key, const NodeData& data) const { return m_raw.save(key, data); }

    // Records a node in the index after write_payload has succeeded for that key.
    // Not thread safe. Fails with AlreadyExists if the node is already physically
    // present in the index.
    Expected<void> commit(const Key& key)
    {
        auto added = m_index->add(key);
        if (!added) {
            return Error::propagate(std::move(added), "add node " + Traits::key_to_string(key) + " to storage index");
        }
        if (!added.value()) {
            return Error::fail(Error::Code::AlreadyExists, "node " + Traits::key_to_string(key) + " is already physically present in the storage index");
        }
        m_dirty = true;
        return {};
    }

    Expected<void> link_from(const Key& key, const Storage& source)
    {
        if (!Traits::is_valid(key)) {
            return store::invalid_key_error<Traits>(key);
        }
        auto exists = has(key);
        if (!exists) {
            return Error::propagate(std::move(exists), "check whether target node " + Traits::key_to_string(key) + " exists before linking");
        }
        if (exists.value()) {
            const auto node_paths = m_raw.paths(key).value();
            return Error::fail(Error::Code::AlreadyExists, "link node to", node_paths.empty() ? m_raw.layout().node_path(key) : node_paths.front());
        }

        auto result = m_raw.link_from(key, source.m_raw);
        if (!result) {
            return result;
        }
        return commit(key);
    }

    Expected<bool> has(const Key& key) const
    {
        if (!Traits::is_valid(key)) {
            return store::invalid_key_error<Traits>(key);
        }
        auto status = m_index->get(key);
        if (!status) {
            return Error::propagate(std::move(status), "look up node " + Traits::key_to_string(key) + " in storage index");
        }
        return status->has_value() && status->value() != NodeStatus::Virtual;
    }

    Expected<std::vector<std::filesystem::path>> paths(const Key& key) const { return m_raw.paths(key); }

    Expected<std::filesystem::path> path_for(const Key& key) const
    {
        auto node_paths = paths(key);
        if (!node_paths) {
            return Error::propagate(std::move(node_paths));
        }
        return node_paths->empty() ? m_raw.layout().node_path(key) : node_paths->front();
    }

    const std::filesystem::path& base_path() const { return m_raw.layout().base_path(); }
    const path_layout::Resolver<Key>& layout() const { return m_raw.layout(); }
    std::optional<std::string_view> codec_selector() const
    {
        if (!m_persistence.has_value()) {
            return std::nullopt;
        }
        return m_persistence->codec_selector;
    }

    bool is_indexed() const { return m_index.has_value(); }

    std::optional<std::reference_wrapper<const Index<Traits>>> index() const
    {
        if (!m_index.has_value()) {
            return std::nullopt;
        }
        return std::cref(m_index.value());
    }

    Expected<void> save_index()
    {
        if (!m_dirty) {
            return {};
        }
        if (!m_index.has_value() || !m_persistence.has_value()) {
            return Error::fail(Error::Code::Internal, "indexed storage has no persistence configuration");
        }
        const IndexMetadata<Traits> metadata {
            m_index.value(),
            m_persistence->layout_id,
            m_persistence->payload_class,
            m_persistence->codec_selector,
        };
        auto result = m_persistence->format.write(m_persistence->index_path, metadata);
        if (result) {
            m_dirty = false;
        }
        return result;
    }

    Expected<void> save_or_create_index() { return save_index(); }

protected:
    Index<Traits>& index_mut() { return m_index.value(); }
    const Index<Traits>& index_ref() const { return m_index.value(); }
    bool is_index_dirty() const { return m_dirty; }
    void set_index_dirty(const bool dirty = true) const { m_dirty = dirty; }

private:
    void finalize_displaced_state() noexcept
    {
        if (!m_dirty || !m_index.has_value()) {
            return;
        }
        const auto result = save_index();
        if (!result) {
            LOG_ERROR("Failed to automatically save index: {}", result.error().to_string());
        }
    }

    RawStorage<Traits, NodeData> m_raw;
    std::optional<Index<Traits>> m_index;
    std::optional<Persistence> m_persistence = std::nullopt;
    mutable bool m_dirty = false;
};

} // namespace store
