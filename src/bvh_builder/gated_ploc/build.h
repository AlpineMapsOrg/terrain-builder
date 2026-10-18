#pragma once

#include <cstdint>
#include <filesystem>
#include <limits>
#include <utility>

#include <libassert/assert.hpp>

#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/DirectoryNotEmpty.h"
#include "io/Error.h"
#include "log.h"

#include "bvh/Storage.h"
#include "bvh/Batch.h"
#include "bvh/Reference.h"
#include "frontier/Storage.h"
#include "frontier/layout.h"
#include "gated_ploc/RoundLayout.h"
#include "gated_ploc/options.h"
#include "gated_ploc/run_round.h"

namespace gated_ploc {

// Writes the gated PLOC tree over the run into `output` and returns its root reference.
// Saves the index of `output`.
// The run must be non-empty and in morton order. It is removed once it has been read.
[[nodiscard]]
inline alp::Result<bvh::Reference, io::Error, io::DirectoryNotEmpty> build(
    frontier::Storage input, bvh::Storage &output,
    const std::filesystem::path &scratch_root, const Options &options) {
    ASSERT(input.size() > 0, "the frontier is empty");

    const auto context = alp::make_context("building a tree from {} to {}", std::filesystem::path(input.directory()), output.directory());

    const detail::RoundLayout layout(scratch_root);
    const size_t stage_count = compute_stage_count(options);
    LOG_INFO("gated ploc runs {} stages per round", stage_count);

    frontier::Storage current = std::move(input);
    uint64_t previous = std::numeric_limits<uint64_t>::max();
    for (uint32_t round = 0;; round++) {
        ALP_TRY_CONTEXT(frontier::Storage next, layout.create_merge_scratch(round), context);
        ALP_TRY_CONTEXT(
            const uint64_t remaining,
            alp::with_context(detail::run_round(current, next, output, options, stage_count), alp::make_context("merging round {}", round)),
            context);
        ASSERT(remaining < previous, "a gated ploc round left the frontier no smaller than it found it");
        previous = remaining;

        ALP_TRY_CONTEXT(next.save_index(), context);
        LOG_DEBUG("gated ploc round {} left {} entries on the frontier", round, remaining);
        ALP_TRY_CONTEXT(std::move(current).remove(), context);

        if (remaining == 1) {
            ALP_TRY_CONTEXT(const frontier::Batch batch, next.read(0), context);
            ALP_TRY_CONTEXT(std::move(next).remove(), context);
            ALP_TRY_CONTEXT(output.save_index(), context);
            return batch.entries.front().ref;
        }

        current = std::move(next);
    }
}

} // namespace gated_ploc
