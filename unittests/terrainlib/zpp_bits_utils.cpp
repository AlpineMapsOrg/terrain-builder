/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Martin Braunsperger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#include "../catch2_helpers.h"
#include "../result_helpers.h"

#include <cstdint>
#include <vector>

#include "io/serde.h"
#include "numeric/number_types.h"

namespace {

template <typename T>
struct Holder {
    T value;
};

template <typename T>
std::vector<uint8_t> serialized(const T &value) {
    return REQUIRE_VALUE(io::serde::serialize(Holder<T>{.value = value}));
}

template <typename T>
T deserialized(const std::vector<uint8_t> &bytes) {
    return REQUIRE_VALUE(io::serde::deserialize<Holder<T>>(bytes)).value;
}

// The same 128 bit value, built the same way in every representation.
constexpr uint64_t high = 0x0123456789abcdefull;
constexpr uint64_t low = 0xfedcba9876543210ull;

template <typename T>
T wide_value() {
    return (T{high} << 64) | T{low};
}

using Wide64 = ::math::wide_integer::uintwide_t<128, std::uint64_t, void, false>;
using Wide32 = ::math::wide_integer::uintwide_t<128, std::uint32_t, void, false>;

}

TEST_CASE("wide integer round trips", "[zpp_bits]") {
    CHECK(deserialized<Wide64>(serialized(wide_value<Wide64>())) == wide_value<Wide64>());
    CHECK(deserialized<Wide32>(serialized(wide_value<Wide32>())) == wide_value<Wide32>());
}

TEST_CASE("wide integer limb widths serialize identically", "[zpp_bits]") {
    CHECK(serialized(wide_value<Wide64>()) == serialized(wide_value<Wide32>()));
}

#if defined(HAS_UINT128)
TEST_CASE("uint128_t round trips", "[zpp_bits]") {
    CHECK(deserialized<uint128_t>(serialized(wide_value<uint128_t>())) == wide_value<uint128_t>());
}

TEST_CASE("uint128_t and wide integers serialize identically", "[zpp_bits]") {
    const std::vector<uint8_t> native = serialized(wide_value<uint128_t>());
    CHECK(native.size() == 16);
    CHECK(native == serialized(wide_value<Wide64>()));
    CHECK(native == serialized(wide_value<Wide32>()));
}
#endif

TEST_CASE("uint_for_bits_t<128> matches both representations", "[zpp_bits]") {
    using MortonCode = uint_for_bits_t<128>;
    const std::vector<uint8_t> selected = serialized(wide_value<MortonCode>());
    CHECK(selected == serialized(wide_value<Wide64>()));
    CHECK(selected == serialized(wide_value<Wide32>()));
}
