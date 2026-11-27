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

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#include <glm/glm.hpp>

#include "../assertion_helpers.h"
#include "../catch2_helpers.h"

#include "numeric/checked_cast.h"

TEST_CASE("is_representable accepts every value the target type can hold", "[numeric][checked_cast]") {
    CHECK(is_representable<uint8_t>(int64_t{255}));
    CHECK(is_representable<uint8_t>(int64_t{0}));
    CHECK(is_representable<int32_t>(uint64_t{2147483647}));
    CHECK(is_representable<int8_t>(int64_t{-128}));
    CHECK(is_representable<uint64_t>(int8_t{127}));
    CHECK(is_representable<int64_t>(uint32_t{4294967295}));
    CHECK(is_representable<uint64_t>(0x1p64 - 0x1p11));
    CHECK(is_representable<int64_t>(-0x1p63));
    CHECK(is_representable<double>(uint64_t{9007199254740992}));
    CHECK(is_representable<double>(uint64_t{0xFFFF'FFFF'FFFF'F800}));
    CHECK(is_representable<float>(0.5));
    CHECK(is_representable<float>(std::numeric_limits<double>::infinity()));
    CHECK(is_representable<float>(std::numeric_limits<double>::quiet_NaN()));
}

TEST_CASE("is_representable rejects every value the target type cannot hold", "[numeric][checked_cast]") {
    CHECK_FALSE(is_representable<uint8_t>(int64_t{256}));
    CHECK_FALSE(is_representable<uint8_t>(int64_t{-1}));
    CHECK_FALSE(is_representable<int32_t>(uint64_t{2147483648}));
    CHECK_FALSE(is_representable<int8_t>(int64_t{-129}));
    CHECK_FALSE(is_representable<uint64_t>(int8_t{-1}));
    CHECK_FALSE(is_representable<uint32_t>(int64_t{4294967296}));
    CHECK_FALSE(is_representable<uint64_t>(0x1p64));
    CHECK_FALSE(is_representable<int64_t>(0x1p63));
    CHECK_FALSE(is_representable<uint32_t>(-1.0));
    CHECK_FALSE(is_representable<int32_t>(0.5));
    CHECK_FALSE(is_representable<int32_t>(std::numeric_limits<double>::quiet_NaN()));
    CHECK_FALSE(is_representable<int64_t>(std::numeric_limits<double>::infinity()));
    CHECK_FALSE(is_representable<double>(uint64_t{9007199254740993}));
    CHECK_FALSE(is_representable<double>(std::numeric_limits<uint64_t>::max()));
    CHECK_FALSE(is_representable<float>(0.1));
    CHECK_FALSE(is_representable<float>(1e300));
}

TEST_CASE("checked_cast keeps a value the target type can hold", "[numeric][checked_cast]") {
    CHECK(checked_cast<uint8_t>(255.0) == 255);
}

TEST_CASE("checked_cast rejects a value the target type cannot hold", "[numeric][checked_cast]") {
    CHECK_ASSERTS(checked_cast<int32_t>(0.5));
}

TEST_CASE("try_cast keeps a value the target type can hold", "[numeric][checked_cast]") {
    CHECK(try_cast<uint8_t>(int64_t{255}) == std::optional<uint8_t>{255});
}

TEST_CASE("try_cast gives nothing for a value the target type cannot hold", "[numeric][checked_cast]") {
    CHECK_FALSE(try_cast<uint8_t>(int64_t{256}).has_value());
}

TEST_CASE("debug_checked_cast keeps a value the target type can hold", "[numeric][checked_cast]") {
    CHECK(debug_checked_cast<uint8_t>(255.0) == 255);
}

#ifndef NDEBUG
TEST_CASE("debug_checked_cast rejects a value the target type cannot hold", "[numeric][checked_cast]") {
    CHECK_ASSERTS(debug_checked_cast<int32_t>(0.5));
}
#endif

TEST_CASE("checked_cast converts a vector whose components all fit", "[numeric][checked_cast]") {
    CHECK(checked_cast<uint8_t>(glm::i64vec3{0, 128, 255}) == glm::u8vec3{0, 128, 255});
    CHECK(checked_cast<int8_t>(glm::u64vec2{0, 127}) == glm::i8vec2{0, 127});
}

TEST_CASE("checked_cast rejects a vector with a single component that does not fit", "[numeric][checked_cast]") {
    CHECK_ASSERTS(checked_cast<uint8_t>(glm::i64vec3{-1, 0, 0}));
    CHECK_ASSERTS(checked_cast<uint8_t>(glm::i64vec3{0, 0, 256}));
}

TEST_CASE("debug_checked_cast converts a vector whose components all fit", "[numeric][checked_cast]") {
    CHECK(debug_checked_cast<uint8_t>(glm::i64vec3{0, 128, 255}) == glm::u8vec3{0, 128, 255});
    CHECK(debug_checked_cast<int8_t>(glm::u64vec2{0, 127}) == glm::i8vec2{0, 127});
}

#ifndef NDEBUG
TEST_CASE("debug_checked_cast rejects a vector with a single component that does not fit", "[numeric][checked_cast]") {
    CHECK_ASSERTS(debug_checked_cast<uint8_t>(glm::i64vec3{-1, 0, 0}));
    CHECK_ASSERTS(debug_checked_cast<uint8_t>(glm::i64vec3{0, 0, 256}));
}
#endif
