#pragma once

#include <cstddef>

// Default maximum batch size in memory.
constexpr size_t DEFAULT_MAX_BATCH_BYTES = 10 * 1024 * 1024;

// Default maximum size of everything kept resident in memory at once.
constexpr size_t DEFAULT_MAX_RESIDENT_BYTES = size_t{4} * 1024 * 1024 * 1024;
