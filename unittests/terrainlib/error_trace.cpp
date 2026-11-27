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

#include <source_location>

#include "error/Trace.h"

TEST_CASE("A trace starts at its origin and grows innermost first", "[error][Trace]") {
    alp::Trace trace(std::source_location::current());
    REQUIRE(trace.frames().size() == 1);
    CHECK(trace.frames()[0].message == std::nullopt);

    trace.push(alp::Frame{.message = "inner", .location = std::source_location::current()});
    trace.push(alp::Frame{.message = "outer", .location = std::source_location::current()});

    REQUIRE(trace.frames().size() == 3);
    CHECK(trace.frames()[1].message == "inner");
    CHECK(trace.frames()[2].message == "outer");
}

TEST_CASE("A trace given a message carries it on its origin", "[error][Trace]") {
    alp::Trace trace(std::source_location::current(), "reading the configuration");

    REQUIRE(trace.frames().size() == 1);
    CHECK(trace.frames()[0].message == "reading the configuration");
}
