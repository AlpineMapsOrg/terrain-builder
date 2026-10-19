#include "../catch2_helpers.h"
#include "zpp_bits_utils.h"

#include <cstdint>
#include <vector>

#include "io/serialize.h"
#include "numeric/number_types.h"

namespace {

template <typename T>
struct Holder {
    T value;
};

template <typename T>
std::vector<uint8_t> serialized(const T &value) {
    auto bytes = io::write_to_bytes(Holder<T>{.value = value});
    REQUIRE(bytes.has_value());
    return bytes.value();
}

template <typename T>
T deserialized(const std::vector<uint8_t> &bytes) {
    auto holder = io::read_from_bytes<Holder<T>>(bytes);
    REQUIRE(holder.has_value());
    return holder.value().value;
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
