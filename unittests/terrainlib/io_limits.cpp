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

#include "../result_helpers.h"
#include "error/prelude.h"
#include "io/limits.h"

TEST_CASE("check_buffer_size computes the required buffer size in bytes", "[io][limits]") {
    const size_t size = REQUIRE_VALUE(io::check_buffer_size<uint32_t>(3, 12));

    CHECK(size == 12);
}

TEST_CASE("check_buffer_size fails for a count whose size size_t cannot hold", "[io][limits]") {
    const auto error = REQUIRE_ERROR(io::check_buffer_size<uint32_t>(size_t{1} << 62, std::numeric_limits<size_t>::max()));

    CHECK(error.holds_code(io::BufferLimitExceeded(io::max_buffer_size)));
}

TEST_CASE("ByteBudget::try_claim counts every claim against one limit", "[io][limits]") {
    io::ByteBudget budget(12);

    REQUIRE_VALUE(budget.try_claim<uint32_t>(2));
    REQUIRE_VALUE(budget.try_claim<uint16_t>(2));
    const auto error = REQUIRE_ERROR(budget.try_claim<uint8_t>(1));

    CHECK(error.holds_code(io::BufferLimitExceeded(12, 13)));
}

TEST_CASE("ByteBudget does not raise the limit above max_buffer_size", "[io][limits]") {
    io::ByteBudget budget(std::numeric_limits<size_t>::max());

    const auto error = REQUIRE_ERROR(budget.try_claim<uint8_t>(io::max_buffer_size + 1));

    CHECK(error.holds_code(io::BufferLimitExceeded(io::max_buffer_size, io::max_buffer_size + 1)));
}

TEST_CASE("ByteBudget::try_claim fails for a count whose size size_t cannot hold", "[io][limits]") {
    io::ByteBudget budget(12);

    const auto error = REQUIRE_ERROR(budget.try_claim<uint32_t>(size_t{1} << 62));

    CHECK(error.holds_code(io::BufferLimitExceeded(12)));
}
