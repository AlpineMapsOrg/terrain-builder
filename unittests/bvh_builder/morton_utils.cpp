#include "../catch2_helpers.h"

#include <cstdint>

#include "range_utils.h"

#include "morton/utils.h"

namespace {

const morton::Code TOP_BIT = morton::Code{1} << (morton::CODE_BITS - 1);

}

TEST_CASE("the common prefix of two codes counts the leading bits they share", "[morton][utils]") {
    const morton::Code zero{0};

    CHECK(morton::count_common_prefix_bits(zero, zero) == morton::CODE_BITS);
    CHECK(morton::count_common_prefix_bits(zero, TOP_BIT) == 0);
    CHECK(morton::count_common_prefix_bits(zero, morton::Code{1}) == morton::CODE_BITS - 1);

    for (const uint32_t bit : range(morton::CODE_BITS)) {
        const morton::Code differing = morton::Code{1} << bit;
        REQUIRE(morton::count_common_prefix_bits(zero, differing) == morton::CODE_BITS - 1 - bit);
    }
}
