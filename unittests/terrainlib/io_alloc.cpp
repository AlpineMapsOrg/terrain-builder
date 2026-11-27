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

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <limits>
#include <new>
#include <ranges>
#include <span>
#include <vector>

#include "../result_helpers.h"
#include "common_error_codes.h"
#include "error/prelude.h"
#include "io/alloc.h"
#include "io/limits.h"

TEST_CASE("try_make_vector counts the limit in bytes", "[io][alloc]") {
    REQUIRE_VALUE(io::try_make_vector<uint32_t>(3, 12));

    const auto error = REQUIRE_ERROR(io::try_make_vector<uint32_t>(3, 11));

    CHECK(error.holds_code(io::BufferLimitExceeded(11, 12)));
}

TEST_CASE("try_make_vector does not raise the limit above max_buffer_size", "[io][alloc]") {
    const auto error = REQUIRE_ERROR(io::try_make_vector<uint8_t>(io::max_buffer_size + 1, std::numeric_limits<size_t>::max()));

    CHECK(error.holds_code(io::BufferLimitExceeded(io::max_buffer_size, io::max_buffer_size + 1)));
}

TEST_CASE("try_to_vector counts the limit in bytes", "[io][alloc]") {
    const std::vector<uint32_t> source{1, 2, 3};

    const auto error = REQUIRE_ERROR(io::try_to_vector(std::span(source), 11));

    CHECK(error.holds_code(io::BufferLimitExceeded(11, 12)));
}

TEST_CASE("try_resize leaves the vector unchanged when it fails", "[io][alloc]") {
    std::vector<uint16_t> vector{1, 2};

    const auto error = REQUIRE_ERROR(io::try_resize(vector, 4, 7));

    CHECK(error.holds_code(io::BufferLimitExceeded(7, 8)));
    CHECK(vector == std::vector<uint16_t>{1, 2});
}

TEST_CASE("try_reserve leaves the vector unchanged when it fails", "[io][alloc]") {
    std::vector<uint8_t> vector;
    const size_t capacity = vector.capacity();

    const auto error = REQUIRE_ERROR(io::try_reserve(vector, 10, 9));

    CHECK(error.holds_code(io::BufferLimitExceeded(9, 10)));
    CHECK(vector.capacity() == capacity);
}

TEST_CASE("catch_out_of_memory fails with OutOfMemory when the function throws std::bad_alloc", "[io][alloc]") {
    SECTION("returning a value") {
        const auto error = REQUIRE_ERROR(io::catch_out_of_memory([]() -> int { throw std::bad_alloc(); }));

        CHECK(error.holds<OutOfMemory>());
    }

    SECTION("returning void") {
        const auto error = REQUIRE_ERROR(io::catch_out_of_memory([] { throw std::bad_alloc(); }));

        CHECK(error.holds<OutOfMemory>());
    }
}

TEST_CASE("try_append_range appends up to the limit", "[io][alloc]") {
    std::vector<uint16_t> vector{1, 2};
    const std::vector<uint16_t> range{3, 4};

    REQUIRE_VALUE(io::try_append_range(vector, range, 8));

    CHECK(vector == std::vector<uint16_t>{1, 2, 3, 4});
}

TEST_CASE("try_append_range counts the elements already in the vector and leaves it unchanged when it fails", "[io][alloc]") {
    std::vector<uint16_t> vector{1, 2};
    const std::vector<uint16_t> range{3, 4};

    const auto error = REQUIRE_ERROR(io::try_append_range(vector, range, 7));

    CHECK(error.holds_code(io::BufferLimitExceeded(7, 8)));
    CHECK(vector == std::vector<uint16_t>{1, 2});
}

TEST_CASE("try_append_range fails when the total size cannot be represented", "[io][alloc]") {
    std::vector<uint8_t> vector{1};

    const auto error = REQUIRE_ERROR(io::try_append_range(vector, std::views::iota(size_t{0}, std::numeric_limits<size_t>::max())));

    CHECK(error.holds_code(io::BufferLimitExceeded(io::max_buffer_size)));
}
