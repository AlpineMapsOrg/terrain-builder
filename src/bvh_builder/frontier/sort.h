#pragma once

#include <algorithm>
#include <span>

#include "frontier/layout.h"

namespace frontier {

// Sorts `entries` in ascending order.
inline void sort_entries(const std::span<Entry> entries) {
    std::ranges::sort(entries);
}

}
