#pragma once

#include <algorithm>
#include <cstddef>

#include "build/merge_cost.h"
#include "frontier/layout.h"
#include "ploc/NearestSearch.h"
#include "storage/batch_limits.h"

namespace gated_ploc {

struct AdmissionOptions {
    // How many entries share one admission limit.
    size_t region_size = 1 << 16;
    // The order statistic a region measures its small nodes by.
    double quantile = 0.33;
    // How many times the extent at `quantile` a node may measure and still merge.
    double tolerance = 3.0;
};

struct Options {
    size_t search_radius = 16;
    double error_weight = build::DEFAULT_ERROR_WEIGHT;
    AdmissionOptions admission = {};

    size_t min_chunk_size = 1 << 12;
    size_t max_batch_bytes = DEFAULT_MAX_BATCH_BYTES;
    size_t max_resident_bytes = DEFAULT_MAX_RESIDENT_BYTES;
};

// The most stages a round runs, since each stage nests the calls of the next on the stack.
constexpr size_t MAX_STAGE_COUNT = 20;

// How many stages fit into `options.max_resident_bytes`.
[[nodiscard]]
inline size_t compute_stage_count(const Options &options) {
    const size_t window_size = options.admission.region_size + options.min_chunk_size + 2 * ploc::halo(options.search_radius);
    return std::clamp(options.max_resident_bytes / (window_size * sizeof(frontier::Entry)), size_t{1}, MAX_STAGE_COUNT);
}

} // namespace gated_ploc
