#pragma once

#include <cstddef>
#include <optional>
#include <span>

#include <libassert/assert.hpp>

#include "error/Result.h"
#include "error/try.h"
#include "io/serde.h"
#include "range_utils.h"

#include "bvh/BatchWriter.h"
#include "frontier/layout.h"
#include "frontier/merge_entry.h"
#include "hploc/options.h"
#include "ploc/NearestSearch.h"

namespace hploc {

namespace detail {

// Merges the clusters of one list, pairing those that are each other's nearest.
class ClusterMerger {
public:
    ClusterMerger(bvh::BatchWriter &sink, const Options &options)
        : _sink(sink),
          _search(options.search_radius, options.error_weight) {}

    // Merges the nearest pairs of `clusters` until at most `limit` are left, which must be at least 1.
    // Returns how many are left. They sit at the front of `clusters` in their original order.
    [[nodiscard]]
    alp::Result<size_t, io::serde::Error> merge_down_to(std::span<frontier::Entry> clusters, const size_t limit) {
        DEBUG_ASSERT(limit >= 1);

        while (clusters.size() > limit) {
            ALP_TRY(const size_t count, this->merge_round(clusters));
            ASSERT(count < clusters.size(), "a round left the clusters no fewer than it found them");
            clusters = clusters.first(count);
        }
        return clusters.size();
    }

private:
    // Merges every pair of `clusters` that are each other's nearest and returns how many are left at its front.
    [[nodiscard]]
    alp::Result<size_t, io::serde::Error> merge_round(const std::span<frontier::Entry> clusters) {
        const std::span<const ploc::Nearest> nearest = this->_search.find_nearest(clusters);

        // The clusters left are compacted to the front in place.
        // The write position never passes the cluster being read, and its partner lies beyond that.
        size_t next_index = 0;
        for (const size_t current : range(clusters.size())) {
            const std::optional<size_t> partner_opt = ploc::find_mutual_partner(nearest, current);

            // A pair is merged once, by its left member.
            if (partner_opt.has_value() && partner_opt.value() < current) {
                continue;
            }

            if (partner_opt.has_value()) {
                ALP_TRY(clusters[next_index], frontier::merge_entries(this->_sink, clusters[current].morton, clusters[current], clusters[partner_opt.value()]));
            } else {
                clusters[next_index] = clusters[current];
            }
            next_index++;
        }
        return next_index;
    }

    bvh::BatchWriter &_sink;
    ploc::NearestSearch _search;
};

} // namespace detail

} // namespace hploc
