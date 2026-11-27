#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "containers/Heap.h"
#include "enumerate.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/serde.h"

#include "frontier/BatchReader.h"
#include "frontier/BatchWriter.h"
#include "frontier/Storage.h"
#include "frontier/layout.h"

namespace sort {

namespace detail {

// The next entry of a run that is being merged.
struct Head {
    frontier::Entry entry;
    uint32_t run;

    auto operator<=>(const Head &) const = default;
};

// Puts the next entry of run `run` on `heads`. Does nothing once `reader` is exhausted.
[[nodiscard]]
inline alp::Result<void, io::serde::Error> push_next_head(frontier::BatchReader &reader, const uint32_t run, MinHeap<Head> &heads) {
    ALP_TRY(const auto entry_opt, reader.next());
    if (entry_opt.has_value()) {
        heads.push(Head{.entry = *entry_opt, .run = run});
    }
    return {};
}

} // namespace detail

// Writes the entries of the sorted `runs` to `output` in Morton order and saves its index.
[[nodiscard]]
inline alp::Result<void, io::serde::Error> merge_runs(const std::span<const frontier::Storage> runs, frontier::Storage &output, const size_t max_batch_bytes) {
    return alp::run_with_context(
        alp::make_context("merging {} runs into {}", runs.size(), output.directory()),
        [&] -> alp::Result<void, io::serde::Error> {
            std::vector<frontier::BatchReader> readers;
            readers.reserve(runs.size());
            for (const frontier::Storage &run : runs) {
                readers.emplace_back(run);
            }

            // Each run is sorted, so the smallest entry not yet written is always the head of some run.
            MinHeap<detail::Head> heads;
            for (const auto &[run, reader] : enumerate<uint32_t>(readers)) {
                ALP_TRY(detail::push_next_head(reader, run, heads));
            }

            frontier::BatchWriter writer(output, max_batch_bytes);
            while (!heads.empty()) {
                const detail::Head head = heads.top();
                heads.pop();

                ALP_TRY(writer.write(head.entry));
                ALP_TRY(detail::push_next_head(readers[head.run], head.run, heads));
            }
            ALP_TRY(writer.flush());

            ALP_TRY(output.save_index());
            return {};
        });
}

}
