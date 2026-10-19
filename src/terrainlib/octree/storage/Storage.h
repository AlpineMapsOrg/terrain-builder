#pragma once

#include <filesystem>
#include <list>
#include <memory>
#include <optional>
#include <source_location>
#include <system_error>
#include <unordered_map>
#include <utility>

#include <libassert/assert.hpp>

#include "error/log.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/Error.h"
#include "log.h"
#include "mesh/io.h"
#include "octree/Id.h"
#include "octree/IndexMap.h"
#include "octree/disk/Layout.h"
#include "octree/disk/layout/strategy/Default.h"
#include "octree/NodeStatusOrMissing.h"
#include "octree/storage/RawStorage.h"
#include "octree/storage/cache/ICache.h"
#include "octree/storage/codec/Codec.h"
#include "octree/storage/defaults.h"
#include "octree/storage/helpers.h"
#include "optional_utils.h"

namespace octree {

namespace detail {
// The error for node `id`, which the index does not contain, originating at the caller.
inline alp::Unexpected<io::FileError> make_index_miss(const Id &id, const disk::Layout &layout, const std::source_location location = std::source_location::current()) {
    const io::FileError error = io::make_file_not_found(layout.get_node_path(id));
    return alp::make_unexpected(error, fmt::format("looking up node {} in the index", id), location);
}

struct MaybeIndex {
    std::optional<IndexMap> map;
    mutable bool dirty = false;

    explicit MaybeIndex() : map(std::nullopt) {}
    explicit MaybeIndex(IndexMap map) : map(std::move(map)) {}

    MaybeIndex(const MaybeIndex &) = default;
    MaybeIndex &operator=(const MaybeIndex &) = default;

    MaybeIndex(MaybeIndex &&other) noexcept
        : map(std::move(other.map)), dirty(std::exchange(other.dirty, false)) {
        other.map.reset();
    }

    MaybeIndex &operator=(MaybeIndex &&other) noexcept {
        this->map = std::move(other.map);
        this->dirty = std::exchange(other.dirty, false);
        other.map.reset();
        return *this;
    }

    bool add(const Id &id) noexcept {
        if (!map.has_value()) {
            return false;
        }

        const bool changed = map->add(id);
        dirty = dirty || changed;
        return changed;
    }

    bool remove(const Id &id) noexcept {
        if (!map.has_value()) {
            return false;
        }

        const bool changed = map->remove(id);
        dirty = dirty || changed;
        return changed;
    }

    bool contains(const Id &id, const bool def = false) const noexcept {
        if (this->map.has_value()) {
            const octree::NodeStatusOrMissing status = this->map->get(id);
            return status == octree::NodeStatusOrMissing::Leaf || status == NodeStatusOrMissing::Inner;
        }
        return def;
    }
};

template <typename T>
struct MaybeCache : public cache::ICache<T> {
    std::optional<std::unique_ptr<cache::ICache<T>>> cache;

    explicit MaybeCache() : cache(std::nullopt) {}
    explicit MaybeCache(std::unique_ptr<cache::ICache<T>> cache) : cache(std::move(cache)) {}

    std::optional<T> get(const Id& id) noexcept override {
        if (this->cache.has_value()) {
            return this->cache->get()->get(id);
        }
        return std::nullopt;
    }

    bool put(const Id &id, const T &value) noexcept override {
        if (this->cache.has_value()) {
            return this->cache->get()->put(id, value);
        }
        return false;
    }

    bool remove(const Id &id) noexcept override {
        if (this->cache.has_value()) {
            return this->cache->get()->remove(id);
        }
        return false;
    }

    bool contains(const Id &id) const noexcept override {
        if (this->cache.has_value()) {
            return this->cache->get()->contains(id);
        }
        return false;
    }
};
}

struct StorageSettings {
    bool allow_overwrite = false;
};

template <typename T = DefaultT, CodecFor<T> Codec = DefaultCodecFor<T>>
class Storage_ {
public:
    using value_type = T;
    using CodecType = Codec;
    using LoadError = alp::Error<io::FileError, typename Codec::LoadError>;
    using SaveError = typename Codec::SaveError;
    using CopyError = typename RawStorage_<T, Codec>::CopyError;

    explicit Storage_(RawStorage_<T, Codec> inner)
        : _inner(std::move(inner)) {}
    explicit Storage_(RawStorage_<T, Codec> inner, IndexMap index)
        : _inner(std::move(inner)), _index(detail::MaybeIndex(std::move(index))) {}

    Storage_ &operator=(const Storage_ &) = delete;
    Storage_(const Storage_ &) = delete;
    Storage_(Storage_ &&) = default;
    Storage_ &operator=(Storage_ &&) = default;

    virtual ~Storage_() {
        if (this->_index.map.has_value() && this->_index.dirty) {
            auto result = helpers::save_index_map(this->_index.map.value(), this->_inner.layout());
            if (!result.has_value()) {
                LOG_UNEXPECTED(result.error(), "closing a storage at {}", this->base_path());
            }
        }
    }

    [[nodiscard]]
    alp::Result<value_type, LoadError> load(const Id &id) const {
        if (const auto value_opt = this->_cache.get(id)) {
            return value_opt.value();
        }

        if (!this->_index.contains(id, true)) {
            return detail::make_index_miss(id, this->_inner.layout());
        }

        const auto result = this->_inner.load(id);
        if (result.has_value()) {
            this->_cache.put(id, result.value());
        }
        return result;
    }

    [[nodiscard]]
    alp::Result<void, SaveError> save(const Id &id, const value_type &value) {
        ASSERT(!this->check_overwrite(id), "tried to overwrite value when not allowed");

        const auto result = this->_inner.save(id, value);
        if (result.has_value()) {
            this->_cache.put(id, value);
            this->_index.add(id);
        }
        return result;
    }

    [[nodiscard]]
    alp::Result<void, CopyError> copy_from(const Id &id, const Storage_<T, Codec> &source) {
        if (!source._index.contains(id, true)) {
            return detail::make_index_miss(id, source._inner.layout());
        }

        ASSERT(!this->check_overwrite(id), "tried to overwrite value when not allowed");

        const auto result = this->_inner.copy_from(id, source._inner);
        if (result.has_value()) {
            this->_index.add(id);
        }
        return result;
    }

    [[nodiscard]]
    alp::Result<void, CopyError> copy_to(const Id &id, Storage_<T, Codec> &target) const {
        return target.copy_from(id, *this);
    }

    bool remove(const Id &id) noexcept {
        this->_cache.remove(id);
        this->_index.remove(id);
        return this->_inner.remove(id);
    }
  
    bool has(const Id &id) const noexcept {
        if (this->is_indexed()) {
            return this->_index.contains(id, false);
        } else {
            return this->_inner.has(id);
        }
    }

    std::filesystem::path path_for(const Id &id) const noexcept {
        return this->_inner.path_for(id);
    }

    std::filesystem::path base_path() const noexcept {
        return this->_inner.base_path();
    }

    bool is_indexed() const noexcept {
        return this->_index.map.has_value();
    }

    [[nodiscard]]
    alp::Result<void, io::FileError> ensure_indexed() noexcept {
        if (this->is_indexed()) {
            return {};
        }

        LOG_TRACE("Index not present, creating empty index");
        IndexMap map;
        ALP_TRY(helpers::update_index_map(map, this->_inner.layout()));
        LOG_TRACE("Index created with {} entries", map.size());
        this->_index.map = std::move(map);
        return {};
    }
    
    std::optional<std::reference_wrapper<const IndexMap>> index() const noexcept {
        return as_ref(this->_index.map);
    }

    std::optional<std::unique_ptr<cache::ICache<T>>> &cache() noexcept {
        return this->_cache.cache;
    }

    std::optional<std::reference_wrapper<const cache::ICache<T>>> cache() const noexcept {
        if (this->_cache.cache.has_value()) {
            return *this->_cache.cache.value();
        } else {
            return std::nullopt;
        }
    }

    [[nodiscard]]
    io::Result<void> save_or_create_index() {
        if (this->is_indexed() && !this->_index.dirty) {
            return {};
        }

        ALP_TRY(this->ensure_indexed());

        auto result = helpers::save_index_map(*ASSERT_VAL(this->index()), this->_inner.layout());
        if (result.has_value()) {
            this->_index.dirty = false;
        }
        return result;
    }

    const octree::disk::Layout &layout() const noexcept {
        return this->_inner.layout();
    }

    const StorageSettings& settings() const noexcept {
        return this->_settings;
    }
    StorageSettings& settings() noexcept {
        return this->_settings;
    }

protected:
    bool check_overwrite(const Id &id) noexcept {
        return !this->_settings.allow_overwrite && this->has(id);
    }

    std::optional<IndexMap> &index_mut() noexcept {
        return this->_index.map;
    }

    bool is_index_dirty() const noexcept {
        return this->_index.dirty;
    }
    void set_index_dirty(const bool val = true) const noexcept {
        this->_index.dirty = val;
    }

    // Rebuilds the index from the stored nodes if the storage has one; drops the index on failure.
    [[nodiscard]]
    alp::Result<void, io::FileError> update_index() noexcept {
        if (!this->is_indexed()) {
            return {};
        }

        IndexMap &index = this->index_mut().value();
        auto result = helpers::update_index_map(index, this->_inner.layout());
        if (!result.has_value()) {
            this->_index.map.reset();
        }
        this->_index.dirty = false;
        return result;
    }

private:
    RawStorage_<T, Codec> _inner;
    detail::MaybeIndex _index = detail::MaybeIndex();
    mutable detail::MaybeCache<T> _cache = detail::MaybeCache<T>();
    StorageSettings _settings = {};
};

} // namespace octree
