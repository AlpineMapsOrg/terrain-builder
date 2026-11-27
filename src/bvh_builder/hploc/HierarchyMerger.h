#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include <libassert/assert.hpp>

#include "error/Result.h"
#include "error/try.h"
#include "io/serde.h"
#include "optional_utils.h"

#include "bvh/BatchWriter.h"
#include "frontier/layout.h"
#include "hploc/ClusterMerger.h"
#include "hploc/Key.h"
#include "hploc/SegmentStack.h"
#include "hploc/options.h"

namespace hploc {

namespace detail {

// Merges the clusters of a run along its morton hierarchy, bottom-up, until one is left.
class HierarchyMerger {
public:
    HierarchyMerger(bvh::BatchWriter &sink, const Options &options)
        : _merger(sink, options),
          _merge_threshold(options.merge_threshold) {}

    // Adds the next entry of the run, which must not come before the last one in morton order.
    [[nodiscard]]
    alp::Result<void, io::serde::Error> push(const frontier::Entry &entry) {
        // The depth of the node splitting this entry from the one before it is how many leading bits their keys share.
        const Key key{.code = entry.morton, .position = this->_entry_count};
        const uint32_t depth = alp::transform_or(this->_previous_key, 0, [&](const Key &previous) {
            return count_common_key_bits(previous, key);
        });
        this->_previous_key = key;
        this->_entry_count++;

        // The range on top is complete once it splits from the range below at least as deep as from this entry.
        while (this->_ranges.size() >= 2 && this->_ranges.top().depth >= depth) {
            ALP_TRY(this->join_top());
        }

        this->_ranges.push(depth, entry);
        return {};
    }

    // Merges everything pushed into one tree and returns its root.
    // At least one entry must have been pushed.
    [[nodiscard]]
    alp::Result<frontier::Entry, io::serde::Error> finish() && {
        ASSERT(this->_ranges.size() > 0, "no entry was pushed");

        while (this->_ranges.size() >= 2) {
            ALP_TRY(this->join_top());
        }
        ALP_TRY(this->merge_top_down_to(1));
        return this->_ranges.top().items.front();
    }

private:
    // Joins the range on top into the one below it, merging their clusters down to the threshold.
    [[nodiscard]]
    alp::Result<void, io::serde::Error> join_top() {
        this->_ranges.join_top();
        return this->merge_top_down_to(this->_merge_threshold);
    }

    // Merges the clusters of the range on top down to at most `limit`.
    [[nodiscard]]
    alp::Result<void, io::serde::Error> merge_top_down_to(const size_t limit) {
        ALP_TRY(const size_t count, this->_merger.merge_down_to(this->_ranges.top().items, limit));
        this->_ranges.shrink_top(count);
        return {};
    }

    ClusterMerger _merger;
    size_t _merge_threshold;
    // The ranges of the run not yet joined with the range before them, each holding its clusters.
    SegmentStack<frontier::Entry> _ranges;
    std::optional<Key> _previous_key;
    // How many entries were pushed, which is the position of the next one.
    uint64_t _entry_count = 0;
};

} // namespace detail

} // namespace hploc
