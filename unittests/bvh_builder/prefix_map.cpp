#include "../catch2_helpers.h"

#include <cstdint>

#include "morton/PrefixMap.h"
#include "morton/code.h"

namespace {

constexpr uint32_t PREFIX_BITS = 3;
constexpr uint32_t CELL_COUNT = 8;

// A code inside the given prefix cell.
morton::Code make_code_in_cell(const uint32_t cell, const uint32_t offset = 0) {
    return (morton::Code{cell} << (morton::CODE_BITS - PREFIX_BITS)) + morton::Code{offset};
}

}

TEST_CASE("a prefix map indexes by the leading bits of the code", "[morton][PrefixMap]") {
    morton::PrefixMap<uint32_t> map(PREFIX_BITS, 0);

    REQUIRE(map.cells().size() == CELL_COUNT);
    CHECK(map.prefix_bits() == PREFIX_BITS);

    map.at(make_code_in_cell(5)) = 42;
    CHECK(map.at(make_code_in_cell(5, 1234)) == 42);
    CHECK(map.at(make_code_in_cell(4)) == 0);
    CHECK(map.cells()[5] == 42);
}
