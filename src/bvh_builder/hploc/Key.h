#pragma once

#include <bit>
#include <cstdint>

#include "morton/code.h"
#include "morton/utils.h"

namespace hploc {

namespace detail {

// An entry's code made unique by its position in the run.
struct Key {
    morton::Code code;
    uint64_t position;
};

// Counts the leading bits both keys share, reading the position as the bits after the code.
[[nodiscard]]
inline uint32_t count_common_key_bits(const Key &a, const Key &b) {
    const uint32_t shared_code_bits = morton::count_common_prefix_bits(a.code, b.code);
    if (shared_code_bits < morton::CODE_BITS) {
        return shared_code_bits;
    }
    return morton::CODE_BITS + static_cast<uint32_t>(std::countl_zero(a.position ^ b.position));
}

} // namespace detail

} // namespace hploc
