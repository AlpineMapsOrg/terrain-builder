#pragma once

#include <cstddef>
#include <cstdint>

#include "build/merge_cost.h"
#include "storage/batch_limits.h"

namespace widen {

// The most children a node may end up with.
constexpr uint32_t MAX_ARITY = 127;

struct Options {
    // The most children a node may end up with (2 <= x <= MAX_ARITY).
    uint32_t arity = 8;
    // The weight the error interval carries against the extent.
    double error_weight = build::DEFAULT_ERROR_WEIGHT;
    size_t max_batch_bytes = DEFAULT_MAX_BATCH_BYTES;
    size_t max_resident_bytes = DEFAULT_MAX_RESIDENT_BYTES;
};

}