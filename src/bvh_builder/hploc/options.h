#pragma once

#include <cstddef>

#include "build/merge_cost.h"
#include "storage/batch_limits.h"

namespace hploc {

struct Options {
    // The most clusters a subtree of the morton hierarchy keeps unmerged when it joins its sibling (x >= 1).
    // Half the wave size of the H-PLOC paper.
    size_t merge_threshold = 16;
    size_t search_radius = 8;
    double error_weight = build::DEFAULT_ERROR_WEIGHT;
    size_t max_batch_bytes = DEFAULT_MAX_BATCH_BYTES;
};

}
