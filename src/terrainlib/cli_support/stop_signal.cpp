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

#include "cli_support/stop_signal.h"

#include <atomic>
#include <csignal>

namespace cli_support::stop_signal {

namespace {
    static_assert(std::atomic<int>::is_always_lock_free);
    std::atomic<int> received = 0;
    void record(int signal) { received.store(signal, std::memory_order_relaxed); }
} // namespace

void install()
{
    std::signal(SIGINT, record);
    std::signal(SIGTERM, record);
}

bool requested() { return received.load(std::memory_order_relaxed) != 0; }

int exit_code()
{
    const auto signal = received.load(std::memory_order_relaxed);
    return signal ? 128 + signal : 1;
}

} // namespace cli_support::stop_signal
