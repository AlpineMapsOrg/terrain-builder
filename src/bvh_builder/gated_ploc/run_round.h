#pragma once

#include <cstdint>
#include <utility>

#include "error/Result.h"
#include "error/try.h"
#include "io/Error.h"
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

// Merges what the round admits from `input`, writing the nodes to `output` and the next run to `next` in morton order.
// Returns how many entries the next run holds.
[[nodiscard]]
inline alp::Result<uint64_t, io::Error> run_round(
    const frontier::Storage &input, frontier::Storage &next, bvh::Storage &output, const Options &options) {
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

    auto [held, admitted] = make_merging_sink<2, frontier::Entry>(std::move(counting_sink));
    ChunkMerger merger(sink, std::move(admitted), options);
    AdmissionGate gate(std::move(merger), std::move(held), options.admission);

    ALP_TRY(frontier::traverse_batches(input, [&gate](const frontier::Batch &batch) {
        return gate.push(batch.entries);
    }));
    ALP_TRY(std::move(gate).finish());
    ALP_TRY(sink.flush());
    return remaining;
}

} // namespace detail

} // namespace gated_ploc
