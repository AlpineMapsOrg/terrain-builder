#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <libassert/assert.hpp>

#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/serde.h"

#include "frontier/Storage.h"
#include "sort/merge_runs.h"
#include "sort/options.h"
#include "sort/split_into_sorted_runs.h"

namespace sort {

namespace detail {

// Merges `runs` in groups of at most `fan_in` into new runs under `directory`.
[[nodiscard]]
inline alp::Result<std::vector<frontier::Storage>, io::serde::Error> merge_round(
    const std::span<const frontier::Storage> runs, const std::filesystem::path &directory, const size_t fan_in, const size_t max_batch_bytes) {
    std::vector<frontier::Storage> merged_runs;
    for (size_t first = 0; first < runs.size(); first += fan_in) {
        const size_t count = std::min(fan_in, runs.size() - first);
        const std::span<const frontier::Storage> group = runs.subspan(first, count);

        ALP_TRY(frontier::Storage merged, create_run_storage(directory, merged_runs.size()));
        ALP_TRY(merge_runs(group, merged, max_batch_bytes));
        merged_runs.push_back(std::move(merged));
    }
    return merged_runs;
}

} // namespace detail

// Writes the entries of `input` to `output` in Morton order, keeping intermediate runs under `scratch` until they are merged.
[[nodiscard]]
inline alp::Result<void, io::serde::Error> sort_by_morton(
    const frontier::Storage &input, frontier::Storage &output, const std::filesystem::path &scratch, const Options &options) {
    DEBUG_ASSERT(options.max_batch_bytes > 0);

    return alp::run_with_context(
        alp::make_context("sorting a frontier from {} to {}", input.directory(), output.directory()),
        [&] -> alp::Result<void, io::serde::Error> {
            // A merge holds one batch per run and one more for its output, and it needs two runs to make progress.
            const size_t resident_batches = options.max_resident_bytes / options.max_batch_bytes;
            const size_t fan_in = std::max<size_t>(resident_batches, 3) - 1;

            ALP_TRY(std::vector<frontier::Storage> runs, split_into_sorted_runs(input, scratch / "split", options.max_resident_bytes, options.max_batch_bytes));

            uint32_t round = 0;
            while (runs.size() > fan_in) {
                const std::filesystem::path directory = scratch / fmt::format("round-{:04}", round);
                ALP_TRY(runs, detail::merge_round(runs, directory, fan_in, options.max_batch_bytes));
                round++;
            }

            return merge_runs(runs, output, options.max_batch_bytes);
        });
}

}
