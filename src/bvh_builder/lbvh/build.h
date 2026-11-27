#pragma once

#include <cstddef>

#include "error/Result.h"
#include "io/serde.h"

#include "bvh/Storage.h"
#include "bvh/Reference.h"
#include "frontier/Storage.h"
#include "hploc/build.h"
#include "hploc/options.h"
#include "storage/batch_limits.h"

namespace lbvh {

struct Options {
    size_t max_batch_bytes = DEFAULT_MAX_BATCH_BYTES;
};

// Writes the LBVH over the run into `output` and returns its root reference.
// Saves the index of `output`.
// The run must be non-empty and in morton order.
[[nodiscard]]
inline alp::Result<bvh::Reference, io::serde::Error> build(const frontier::Storage &input, bvh::Storage &output, const Options &options) {
    return hploc::build(input, output, hploc::Options{
        .merge_threshold = 1,
        .max_batch_bytes = options.max_batch_bytes,
    });
}

} // namespace lbvh
