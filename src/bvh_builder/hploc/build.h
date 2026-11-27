#pragma once

#include <utility>

#include <libassert/assert.hpp>

#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/serde.h"

#include "bvh/BatchWriter.h"
#include "bvh/Storage.h"
#include "bvh/Reference.h"
#include "frontier/Storage.h"
#include "frontier/layout.h"
#include "frontier/traverse.h"
#include "hploc/HierarchyMerger.h"
#include "hploc/options.h"

namespace hploc {

// Writes the H-PLOC tree over the run through `sink` and returns its root.
// The run must be non-empty and in morton order.
[[nodiscard]]
inline alp::Result<frontier::Entry, io::serde::Error> build(const frontier::Storage &input, bvh::BatchWriter &sink, const Options &options) {
    ASSERT(input.size() > 0);

    detail::HierarchyMerger merger(sink, options);
    ALP_TRY(frontier::traverse_entries(input, [&](const frontier::Entry &entry) -> alp::Result<void, io::serde::Error> {
        return merger.push(entry);
    }));
    return std::move(merger).finish();
}

// Writes the H-PLOC tree over the run into `output` and returns its root reference.
// Saves the index of `output`.
// The run must be non-empty and in morton order.
[[nodiscard]]
inline alp::Result<bvh::Reference, io::serde::Error> build(const frontier::Storage &input, bvh::Storage &output, const Options &options) {
    return alp::run_with_context(
        alp::make_context("building a tree from {} to {}", input.directory(), output.directory()),
        [&] -> alp::Result<bvh::Reference, io::serde::Error> {
            bvh::BatchWriter sink(output, options.max_batch_bytes);

            ALP_TRY(const frontier::Entry root, build(input, sink, options));
            ALP_TRY(sink.flush());
            ALP_TRY(output.save_index());
            return root.ref;
        });
}

} // namespace hploc
