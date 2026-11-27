#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <iterator>
#include <limits>
#include <span>
#include <vector>

#include <fmt/format.h>
#include <libassert/assert.hpp>

#include "io/serde.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"

#include "storage/batch_limits.h"
#include "bvh/BatchWriter.h"
#include "bvh/Storage.h"
#include "bvh/Batch.h"
#include "bvh/Reference.h"
#include "frontier/Storage.h"
#include "frontier/layout.h"
#include "frontier/merge_entry.h"
#include "frontier/traverse.h"
#include "morton/code.h"
#include "morton/utils.h"

namespace pseudo_lbvh {

struct Options {
    size_t max_batch_bytes = DEFAULT_MAX_BATCH_BYTES;
};

namespace detail {

// Keeps the code's leading `bits` bits and clears the rest.
[[nodiscard]]
inline morton::Code extract_prefix(const morton::Code &code, const uint32_t bits) {
    if (bits == 0) {
        return morton::Code{0};
    }

    const morton::Code mask = std::numeric_limits<morton::Code>::max() << (morton::CODE_BITS - bits);
    return code & mask;
}

// Finds where the run divides at the highest bit its codes disagree on, or in half when they all agree.
// Never leaves either side empty.
[[nodiscard]]
inline size_t find_split(const std::span<const frontier::Entry> entries) {
    DEBUG_ASSERT(entries.size() >= 2);

    const uint32_t shared_bits = morton::count_common_prefix_bits(entries.front().morton, entries.back().morton);
    if (shared_bits == morton::CODE_BITS) {
        return entries.size() / 2;
    }

    const morton::Code prefix = extract_prefix(entries.front().morton, shared_bits);
    const morton::Code target = prefix | (morton::Code{1} << (morton::CODE_BITS - shared_bits - 1));
    const auto first_right = std::ranges::lower_bound(entries, target, {}, &frontier::Entry::morton);
    return static_cast<size_t>(std::distance(entries.begin(), first_right));
}

// Writes the subtree over `entries` and returns its root, which carries the code of the first entry.
// `entries` must be non-empty and in morton order.
[[nodiscard]]
inline alp::Result<frontier::Entry, io::serde::Error> build_subtree(
    const std::span<const frontier::Entry> entries, bvh::BatchWriter &sink) {
    ASSERT(!entries.empty());

    if (entries.size() == 1) {
        return entries.front();
    }

    const size_t split = find_split(entries);
    ALP_TRY(const frontier::Entry left, build_subtree(entries.first(split), sink));
    ALP_TRY(const frontier::Entry right, build_subtree(entries.subspan(split), sink));
    return frontier::merge_entries(sink, left.morton, left, right);
}

} // namespace detail

// Writes a subtree over each batch of the run and one over their roots through `sink`, and returns the root.
// The run must be non-empty and in morton order.
[[nodiscard]]
inline alp::Result<frontier::Entry, io::serde::Error> build(
    const frontier::Storage &input, bvh::BatchWriter &sink) {
    ASSERT(input.size() > 0);

    std::vector<frontier::Entry> batch_roots;
    batch_roots.reserve(input.size());

    ALP_TRY(frontier::traverse_batches(input, [&](const frontier::Batch &batch) -> alp::Result<void, io::serde::Error> {
        ALP_TRY(const frontier::Entry root, detail::build_subtree(batch.entries, sink));
        batch_roots.push_back(root);
        return {};
    }));

    return detail::build_subtree(batch_roots, sink);
}

// Writes a subtree over each batch of the run and one over their roots into `output`, and returns the root reference.
// Saves the index of `output`.
// The run must be non-empty and in morton order.
[[nodiscard]]
inline alp::Result<bvh::Reference, io::serde::Error> build(
    const frontier::Storage &input, bvh::Storage &output, const Options &options) {
    return alp::run_with_context(
        alp::make_context("building a tree from {} to {}", input.directory(), output.directory()),
        [&] -> alp::Result<bvh::Reference, io::serde::Error> {
            bvh::BatchWriter sink(output, options.max_batch_bytes);

            ALP_TRY(const frontier::Entry root, build(input, sink));
            ALP_TRY(sink.flush());
            ALP_TRY(output.save_index());
            return root.ref;
        });
}

} // namespace pseudo_lbvh
