#pragma once

#include <filesystem>
#include <shared_mutex>
#include <type_traits>

#include "error/Result.h"
#include "octree/Id.h"

namespace dag {

// A mutex-based thread-safe wrapper around a Storage_ instance.
// TODO: move this locking into Storage_ itself and remove this wrapper once that lands.
template <typename Storage>
class ThreadSafeStorage {
    static_assert(!std::is_const_v<Storage>, "ThreadSafeStorage requires a non-const Storage");

public:
    using value_type = typename Storage::value_type;
    using LoadError = typename Storage::LoadError;
    using SaveError = typename Storage::SaveError;

    explicit ThreadSafeStorage(Storage &&storage)
        : _storage(std::move(storage)) {}

    ThreadSafeStorage(const ThreadSafeStorage &) = delete;
    ThreadSafeStorage &operator=(const ThreadSafeStorage &) = delete;
    ThreadSafeStorage(ThreadSafeStorage &&) = delete;
    ThreadSafeStorage &operator=(ThreadSafeStorage &&) = delete;

    Storage release() && {
        return std::move(this->_storage);
    }

    [[nodiscard]]
    alp::Result<value_type, LoadError> load(const octree::Id &id) const {
        std::shared_lock lock(this->_mutex);
        return this->_storage.load(id);
    }

    bool has(const octree::Id &id) const {
        std::shared_lock lock(this->_mutex);
        return this->_storage.has(id);
    }

    std::filesystem::path base_path() const noexcept {
        return this->_storage.base_path();
    }

    [[nodiscard]]
    alp::Result<void, SaveError> save(const octree::Id &id, const value_type &value) const {
        std::unique_lock lock(this->_mutex);
        return this->_storage.save(id, value);
    }

    [[nodiscard]]
    auto save_or_create_index() const {
        std::unique_lock lock(this->_mutex);
        return this->_storage.save_or_create_index();
    }

private:
    mutable Storage _storage;
    mutable std::shared_mutex _mutex;
};

} // namespace dag
