#pragma once

#include <cstddef>

#include "build/merge_cost.h"
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
};

} // namespace gated_ploc
