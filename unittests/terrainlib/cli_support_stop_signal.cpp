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

#include <csignal>

#include "cli_support/stop_signal.h"

TEST_CASE("install keeps the stop handlers in place until the last guard is destroyed", "[cli_support][stop_signal]") {
    const auto catch_interrupt = std::signal(SIGINT, SIG_IGN);
    const auto catch_terminate = std::signal(SIGTERM, SIG_DFL);

    cli_support::stop_signal::Guard first = cli_support::stop_signal::install();
    cli_support::stop_signal::Guard second = cli_support::stop_signal::install();
    first.reset();
    CHECK(std::signal(SIGINT, SIG_DFL) != SIG_IGN);
    CHECK(std::signal(SIGTERM, SIG_IGN) != SIG_DFL);
    second.reset();

    CHECK(std::signal(SIGINT, catch_interrupt) == SIG_IGN);
    CHECK(std::signal(SIGTERM, catch_terminate) == SIG_DFL);
}

TEST_CASE("install installs again once the earlier guards are destroyed", "[cli_support][stop_signal]") {
    const auto catch_interrupt = std::signal(SIGINT, SIG_IGN);
    const auto catch_terminate = std::signal(SIGTERM, SIG_DFL);

    cli_support::stop_signal::Guard first = cli_support::stop_signal::install();
    first.reset();
    cli_support::stop_signal::Guard second = cli_support::stop_signal::install();
    CHECK(std::signal(SIGINT, SIG_DFL) != SIG_IGN);
    CHECK(std::signal(SIGTERM, SIG_IGN) != SIG_DFL);
    second.reset();

    CHECK(std::signal(SIGINT, catch_interrupt) == SIG_IGN);
    CHECK(std::signal(SIGTERM, catch_terminate) == SIG_DFL);
}
