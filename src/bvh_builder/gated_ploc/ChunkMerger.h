#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "containers/PaddedSpan.h"
#include "error/Result.h"
#include "error/try.h"
#include "io/serde.h"
#include "range_utils.h"

#include "bvh/BatchWriter.h"
#include "frontier/EntrySink.h"
#include "frontier/layout.h"
#include "frontier/merge_entry.h"
#include "gated_ploc/options.h"
#include "ploc/NearestSearch.h"

namespace gated_ploc {

namespace detail {

// Merges the pairs `chunk` owns into nodes written to `sink`, appends the entries it owns to `next_run`, and returns how many pairs it merged.
// Each parent takes its left member's code, so the entries leave in the order of `chunk`.
// `chunk` must carry a halo on each side, except at the ends of the stream.
[[nodiscard]]
inline alp::Result<uint64_t, io::serde::Error> merge_chunk(
    ploc::NearestSearch &search,
    bvh::BatchWriter &sink,
    const PaddedSpan<const frontier::Entry> chunk,
    std::vector<frontier::Entry> &next_run) {
    const std::span<const frontier::Entry> window = chunk.span();
    const std::span<const ploc::Nearest> nearest = search.find_nearest(window);

    const size_t owned_first = chunk.left_padding();
    const size_t owned_last = owned_first + chunk.size();

    uint64_t merges = 0;
    for (const size_t current : range(owned_first, owned_last)) {
        const std::optional<size_t> partner_opt = ploc::find_mutual_partner(nearest, current);
        if (!partner_opt.has_value()) {
            next_run.push_back(window[current]);
            continue;
        }
        // The chunk holding the left member of a pair is the one that writes it.
        if (partner_opt.value() < current) {
            continue;
        }

        const frontier::Entry &left = window[current];
        const frontier::Entry &right = window[partner_opt.value()];
        ALP_TRY(const frontier::Entry merged, frontier::merge_entries(sink, left.morton, left, right));
        next_run.push_back(merged);
        merges++;
    }
    return merges;
}

// Merges a stream of entries in chunks of at least `min_chunk_size`, passing the entries on to `output` in input order.
template <frontier::EntrySink Output>
class ChunkMerger {
public:
    ChunkMerger(bvh::BatchWriter &sink, Output output, const Options &options)
        : _sink(sink),
          _output(std::forward<Output>(output)),
          _search(options.search_radius, options.error_weight),
          _min_chunk_size(options.min_chunk_size) {}

    // Takes the next entries, merging every chunk they complete.
    [[nodiscard]]
    alp::Result<void, io::serde::Error> push(const std::span<const frontier::Entry> entries) {
        this->_pending.append_range(entries);
        return this->merge_pending(false);
    }

    // Merges the entries left at the end of the stream, then ends the output's stream.
    [[nodiscard]]
    alp::Result<void, io::serde::Error> finish() && {
        ALP_TRY(this->merge_pending(true));
        return std::move(this->_output).finish();
    }

private:
    // Merges the pending entries that have their halo, or all of them at the end of the stream, keeping the halo of the next chunk.
    [[nodiscard]]
    alp::Result<void, io::serde::Error> merge_pending(const bool is_last) {
        const size_t halo = ploc::halo(this->_search.search_radius());
        const size_t right_padding = is_last ? 0 : halo;
        const size_t min_size = is_last ? 1 : this->_min_chunk_size;
        if (this->_pending.size() < this->_left_padding + min_size + right_padding) {
            return {};
        }

        const PaddedSpan<const frontier::Entry> chunk(this->_pending, this->_left_padding, right_padding);
        ALP_TRY(merge_chunk(this->_search, this->_sink, chunk, this->_next_run));
        ALP_TRY(this->_output.push(this->_next_run));
        this->_next_run.clear();

        const size_t merged_count = this->_left_padding + chunk.size();
        const size_t left_padding = std::min(merged_count, halo);
        this->_pending.erase(this->_pending.begin(), this->_pending.begin() + (merged_count - left_padding));
        this->_left_padding = left_padding;
        return {};
    }

    bvh::BatchWriter &_sink;
    Output _output;
    ploc::NearestSearch _search;
    size_t _min_chunk_size;
    // The left padding of the next chunk, followed by the entries not merged yet.
    std::vector<frontier::Entry> _pending;
    size_t _left_padding = 0;
    // The entries a chunk passes on, kept to reuse their memory.
    std::vector<frontier::Entry> _next_run;
};

} // namespace detail

} // namespace gated_ploc
