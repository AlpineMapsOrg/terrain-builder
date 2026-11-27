#pragma once

#include <cstddef>

#include "storage/batch_limits.h"

namespace sort {

struct Options {
    size_t max_batch_bytes = DEFAULT_MAX_BATCH_BYTES;
    size_t max_resident_bytes = DEFAULT_MAX_RESIDENT_BYTES;
};

}
