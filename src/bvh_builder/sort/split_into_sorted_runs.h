#pragma once

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <libassert/assert.hpp>

#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/serde.h"
#include "mem.h"

#include "frontier/BatchReader.h"
#include "frontier/BatchWriter.h"
#include "frontier/Storage.h"
#include "frontier/layout.h"
#include "frontier/sort.h"

namespace sort {

namespace detail {

// Starts run `index` as an empty storage under `directory`.
[[nodiscard]]
inline alp::Result<frontier::Storage, io::FileError> create_run_storage(const std::filesystem::path &directory, const size_t index) {
    return frontier::Storage::create(directory / fmt::format("run-{:04}", index));
}

// Takes entries from `reader` until they hold `max_run_bytes`. Fewer once the reader is exhausted.
[[nodiscard]]
inline alp::Result<std::vector<frontier::Entry>, io::serde::Error> read_run_entries(frontier::BatchReader &reader, const size_t max_run_bytes) {
    std::vector<frontier::Entry> entries;
    while (sizeof_val(std::span{entries}) < max_run_bytes) {
        ALP_TRY(const std::optional<frontier::Entry> entry_opt, reader.next());
        if (!entry_opt.has_value()) {
            break;
        }
        entries.push_back(*entry_opt);
    }
    return entries;
}

// Sorts `entries` by Morton code, writes them to `output` and saves its index.
[[nodiscard]]
inline alp::Result<void, io::serde::Error> write_sorted_run(const std::span<frontier::Entry> entries, frontier::Storage &output, const size_t max_batch_bytes) {
    frontier::sort_entries(entries);

    frontier::BatchWriter writer(output, max_batch_bytes);
    for (const frontier::Entry &entry : entries) {
        ALP_TRY(writer.write(entry));
    }
    ALP_TRY(writer.flush());

    return output.save_index();
}

} // namespace detail

// Splits `input` into runs of at most `max_run_bytes` each, sorted by Morton code, every one a new storage under `directory`.
[[nodiscard]]
inline alp::Result<std::vector<frontier::Storage>, io::serde::Error> split_into_sorted_runs(
    const frontier::Storage &input, const std::filesystem::path &directory, const size_t max_run_bytes, const size_t max_batch_bytes) {
    ASSERT(max_run_bytes > 0);

    return alp::run_with_context(
        alp::make_context("splitting {} into sorted runs", input.directory()),
        [&] -> alp::Result<std::vector<frontier::Storage>, io::serde::Error> {
            std::vector<frontier::Storage> runs;

            // The reader keeps its place between runs, so a run that ends inside a batch leaves the rest of it to the next one.
            frontier::BatchReader reader(input);
            while (true) {
                ALP_TRY(std::vector<frontier::Entry> entries, detail::read_run_entries(reader, max_run_bytes));
                if (entries.empty()) {
                    break;
                }

                ALP_TRY(frontier::Storage run, detail::create_run_storage(directory, runs.size()));
                ALP_TRY(detail::write_sorted_run(entries, run, max_batch_bytes));
                runs.push_back(std::move(run));
            }

            return runs;
        });
}

}
