#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <functional>
#include <list>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include "error/try.h"

// A cache of values by key, evicting the least recently used once their total cost exceeds the budget.
template <typename Key, typename Value>
class BoundedCache {
public:
    // A value along with how much of the budget it takes up.
    struct Entry {
        Value value;
        size_t cost;
    };

    explicit BoundedCache(const size_t budget) : _budget(budget) {}

    // The value at `key`, taken from `read` if it is not resident, valid until the next call. A failed read caches nothing.
    template <typename F>
    [[nodiscard]]
    std::expected<std::reference_wrapper<const Value>, typename std::invoke_result_t<F &>::error_type> get_or_try_insert_with(const Key &key, F &&read) {
        const auto resident = this->_resident.find(key);
        if (resident != this->_resident.end()) {
            this->_usage.splice(this->_usage.begin(), this->_usage, resident->second.usage_position);
            return std::cref(resident->second.entry.value);
        }

        ALP_TRY(Entry entry, read());
        this->_misses++;
        this->_resident_cost += entry.cost;

        // The entry goes in as the most recently used, so eviction can never take the one just read.
        this->_usage.push_front(key);
        const auto admitted = this->_resident.emplace(key, Resident{.entry = std::move(entry), .usage_position = this->_usage.begin()}).first;
        this->evict_until_within_budget();
        return std::cref(admitted->second.entry.value);
    }

    // How many values are resident.
    uint32_t count_resident() const {
        return static_cast<uint32_t>(this->_resident.size());
    }

    // How many calls had to read.
    uint32_t count_misses() const {
        return this->_misses;
    }

private:
    // The keys of the resident values, most recently used first.
    using Usage = std::list<Key>;

    struct Resident {
        Entry entry;
        typename Usage::iterator usage_position;
    };

    // Evicts values until the resident set is within budget.
    void evict_until_within_budget() {
        while (this->_resident_cost > this->_budget && this->_resident.size() > 1) {
            const auto evicted = this->_resident.find(this->_usage.back());
            this->_resident_cost -= evicted->second.entry.cost;
            this->_resident.erase(evicted);
            this->_usage.pop_back();
        }
    }

    size_t _budget;
    size_t _resident_cost = 0;
    uint32_t _misses = 0;
    Usage _usage;
    std::unordered_map<Key, Resident> _resident;
};
