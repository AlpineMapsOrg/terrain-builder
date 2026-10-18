#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <utility>

#include <libassert/assert.hpp>

#include "error/Result.h"
#include "error/try.h"
#include "io/Error.h"
#include "range_utils.h"
#include "sink/ErasedSink.h"
#include "sink/FnSink.h"
#include "sink/MergingSink.h"

#include "bvh/BatchWriter.h"
#include "bvh/Storage.h"
#include "frontier/BatchWriter.h"
#include "frontier/BatchWriterSink.h"
#include "frontier/Storage.h"
#include "frontier/layout.h"
#include "frontier/traverse.h"
#include "gated_ploc/AdmissionGate.h"
#include "gated_ploc/ChunkMerger.h"
#include "gated_ploc/options.h"

namespace gated_ploc {

namespace detail {

// Admits and merges a stream of entries, passing them on to `output`.
template <typename Output>
auto make_stage(bvh::BatchWriter &sink, Output &&output, const Options &options) {
    auto [held, admitted] = make_merging_sink<2, frontier::Entry>(std::forward<Output>(output));
    ChunkMerger merger(sink, std::move(admitted), options);
    return AdmissionGate(std::move(merger), std::move(held), options.admission);
}

// Merges `input` through the stages into `next`, returning how many entries were written to `next`.
[[nodiscard]]
inline alp::Result<uint64_t, io::Error> run_round(
    const frontier::Storage &input, frontier::Storage &next, bvh::Storage &output, const Options &options, const size_t stage_count) {
    ASSERT(stage_count > 0, "a round needs at least one stage");

    bvh::BatchWriter sink(output, options.max_batch_bytes);
    frontier::BatchWriter next_run(next, options.max_batch_bytes);
    frontier::BatchWriterSink next_run_sink(next_run);
    uint64_t remaining = 0;
    FnSink counting_sink(
        [&](const frontier::Entry &entry) {
            auto result = next_run_sink.push(entry);
            remaining++;
            return result;
        },
        [&] {
            return std::move(next_run_sink).finish();
        });

    // Built from the last stage to the first, each holding the next one by reference.
    std::deque<ErasedSink<frontier::Entry, io::Error>> stages;
    stages.emplace_front(make_stage(sink, std::move(counting_sink), options));
    for ([[maybe_unused]] const size_t i : range(size_t{1}, stage_count)) {
        stages.emplace_front(make_stage(sink, stages.front(), options));
    }

    ErasedSink<frontier::Entry, io::Error> &first = stages.front();
    ALP_TRY(frontier::traverse_entries(input, [&first](const frontier::Entry &entry) {
        return first.push(entry);
    }));
    ALP_TRY(std::move(first).finish());
    ALP_TRY(sink.flush());
    return remaining;
}

} // namespace detail

} // namespace gated_ploc
