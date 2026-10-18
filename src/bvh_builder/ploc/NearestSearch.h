#pragma once

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <limits>
#include <optional>
#include <span>
#include <vector>

#include <radix/geometry.h>

#include "range_utils.h"

#include "build/merge_cost.h"
#include "frontier/layout.h"

namespace ploc {

// The cheapest partner found for an entry, and what it costs.
struct Nearest {
    std::optional<size_t> index;
    double cost = std::numeric_limits<double>::infinity();

    // Takes `partner` if it is cheaper than the one held, reporting whether it did.
    bool offer(const size_t partner, const double cost) {
        if (cost < this->cost) {
            this->index = partner;
            this->cost = cost;
            return true;
        }
        return false;
    }
};

// Finds the cheapest partner for every entry among the `search_radius` entries on either side of it.
class NearestSearch {
public:
    NearestSearch(const size_t search_radius, const double error_weight)
        : _search_radius(search_radius),
          _error_weight(error_weight) {}

    [[nodiscard]]
    size_t search_radius() const {
        return this->_search_radius;
    }

    // Finds the cheapest partner of every one of `entries`, or none for an entry with nothing in reach.
    // Of equally cheap partners it keeps the one with the lowest index. The result is valid until the next call.
    [[nodiscard]]
    std::span<const Nearest> find_nearest(const std::span<const frontier::Entry> entries) {
        const std::span<const radix::geometry::Aabb<4, double>> bounds = this->collect_cost_bounds(entries);
        this->_nearest.assign(bounds.size(), Nearest{});

        // The cost is symmetric, so each pair is measured once and offered to both of its members.
        for (const size_t left : range(bounds.size())) {
            const size_t last = std::min(bounds.size(), left + this->_search_radius + 1);
            for (const size_t right : range(left + 1, last)) {
                const double cost = build::compute_merge_cost(bounds[left], bounds[right]);
                this->_nearest[left].offer(right, cost);
                this->_nearest[right].offer(left, cost);
            }
        }
        return this->_nearest;
    }

private:
    // Collects the cost bounds of every one of `entries`.
    // Valid until the next call.
    [[nodiscard]]
    std::span<const radix::geometry::Aabb<4, double>> collect_cost_bounds(const std::span<const frontier::Entry> entries) {
        this->_bounds.clear();
        std::ranges::transform(entries, std::back_inserter(this->_bounds), [this](const frontier::Entry &e) {
            return build::make_cost_bounds(e.header, this->_error_weight);
        });
        return this->_bounds;
    }

    size_t _search_radius;
    double _error_weight;
    std::vector<radix::geometry::Aabb<4, double>> _bounds;
    std::vector<Nearest> _nearest;
};

// Finds the partner of `entry`, or nothing unless the two are each other's nearest.
[[nodiscard]]
inline std::optional<size_t> find_mutual_partner(const std::span<const Nearest> nearest, const size_t entry) {
    const std::optional<size_t> partner = nearest[entry].index;
    if (partner.has_value() && nearest[partner.value()].index == entry) {
        return partner;
    }
    return std::nullopt;
}

// How many entries a run needs on each side so its mutual partners match those of the whole stream.
[[nodiscard]]
inline size_t halo(const size_t search_radius) {
    return 2 * search_radius;
}

} // namespace ploc
