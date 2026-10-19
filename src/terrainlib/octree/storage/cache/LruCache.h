#pragma once

#include <cstddef>
#include <optional>
#include <list>
#include <unordered_map>
#include <utility>

#include "octree/Id.h"
#include "octree/storage/cache/ICache.h"

namespace octree::cache {

// TODO: UNTESTED
template <typename T>
class Lru : public ICache<T> {
public:
    explicit Lru(const size_t capacity) : _capacity(capacity) {}

    std::optional<T> get(const Id &id) noexcept override {
        auto it = this->_map.find(id);
        if (it == this->_map.end()) {
            return std::nullopt;
        }

        // Move to front (most recently used)
        this->_usage.splice(this->_usage.begin(), this->_usage, it->second.second);
        return it->second.first;
    }

    bool put(const Id &id, const T &value) noexcept override {
        if (this->_capacity == 0) {
            return false;
        }

        auto it = this->_map.find(id);
        if (it != this->_map.end()) {
            // Update value and move to front
            it->second.first = value;
            this->_usage.splice(this->_usage.begin(), this->_usage, it->second.second);
        } else {
            // Insert new entry
            if (this->_map.size() == this->_capacity) {
                // Evict least recently used
                const Id &lru_id = this->_usage.back();
                this->_map.erase(lru_id);
                this->_usage.pop_back();
            }

            this->_usage.push_front(id);
            this->_map.emplace(id, std::pair(value, this->_usage.begin()));
        }
        return true;
    }

    bool remove(const Id &id) noexcept override {
        auto it = this->_map.find(id);
        if (it == this->_map.end()) {
            return false;
        }

        this->_usage.erase(it->second.second);
        this->_map.erase(it);
        return true;
    }

    bool contains(const Id &id) const noexcept override {
        return this->_map.contains(id);
    }

private:
    size_t _capacity;
    std::list<Id> _usage;
    std::unordered_map<Id, std::pair<T, typename std::list<Id>::iterator>> _map;
};

}
