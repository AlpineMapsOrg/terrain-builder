/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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

#include <csignal>

#include <catch2/catch_test_macros.hpp>

#include "cli_support/stop_signal.h"

TEST_CASE("stop signals are recorded and mapped to the conventional exit code", "[cli_support]")
{
    cli_support::stop_signal::install();
    CHECK_FALSE(cli_support::stop_signal::requested());
    CHECK(cli_support::stop_signal::exit_code() == 1);

    REQUIRE(std::raise(SIGTERM) == 0);
    CHECK(cli_support::stop_signal::requested());
    CHECK(cli_support::stop_signal::exit_code() == 128 + SIGTERM);
}
