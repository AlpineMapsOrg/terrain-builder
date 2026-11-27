/*****************************************************************************
 * Alpine Terrain Builder
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
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <stop_token>
#include <string>
#include <vector>

#include <fmt/format.h>

#include "../result_helpers.h"
#include "cli_support/for_each_with_progress.h"
#include "error/Error.h"
#include "error/Result.h"
#include "parallel.h"
#include "stop.h"

namespace {

struct FailedItem {
    uint32_t item;
};

std::string format_as(const FailedItem &failed) {
    return fmt::format("item {} failed", failed.item);
}

} // namespace

TEST_CASE("try_for_each_with_progress runs the body once for every item", "[cli_support][for_each_with_progress]") {
    const parallel::Execution execution = GENERATE(parallel::Execution::Parallel, parallel::Execution::Serial);
    const std::vector<uint32_t> items = {3, 5, 8, 13};

    // Catch2 assertions are not thread safe, so the body only records what it saw.
    std::mutex seen_mutex;
    std::vector<uint32_t> seen;
    REQUIRE_VALUE(cli_support::try_for_each_with_progress(items, [&](const uint32_t &item) -> alp::Result<void, FailedItem> {
        const std::lock_guard lock(seen_mutex);
        seen.push_back(item);
        return {};
    }, std::stop_token(), execution));

    std::ranges::sort(seen);
    CHECK(seen == std::vector<uint32_t>{3, 5, 8, 13});
}

TEST_CASE("try_for_each_with_progress returns the error of the body that failed", "[cli_support][for_each_with_progress]") {
    const parallel::Execution execution = GENERATE(parallel::Execution::Parallel, parallel::Execution::Serial);
    const std::vector<uint32_t> items = {3, 5, 8, 13};

    const auto error = REQUIRE_ERROR(cli_support::try_for_each_with_progress(items, [&](const uint32_t &item) -> alp::Result<void, FailedItem> {
        if (item == 8) {
            return alp::make_error(FailedItem{item});
        }
        return {};
    }, std::stop_token(), execution));

    const FailedItem *failed = error.get_if<FailedItem>();
    REQUIRE(failed != nullptr);
    CHECK(failed->item == 8);
}

// No item runs, so the bar is left with every step unfinished.
TEST_CASE("try_for_each_with_progress returns Stopped once the stop was requested before the loop", "[cli_support][for_each_with_progress]") {
    const parallel::Execution execution = GENERATE(parallel::Execution::Parallel, parallel::Execution::Serial);
    const std::vector<uint32_t> items = {3, 5, 8, 13};
    std::stop_source stop;
    stop.request_stop();

    std::atomic<bool> ran = false;
    const auto error = REQUIRE_ERROR(cli_support::try_for_each_with_progress(items, [&](const uint32_t &) -> alp::Result<void, FailedItem> {
        ran = true;
        return {};
    }, stop.get_token(), execution));

    CHECK(error.holds<Stopped>());
    CHECK_FALSE(ran);
}

TEST_CASE("for_each_with_progress runs the body once for every item", "[cli_support][for_each_with_progress]") {
    const parallel::Execution execution = GENERATE(parallel::Execution::Parallel, parallel::Execution::Serial);
    const std::vector<uint32_t> items = {3, 5, 8, 13};

    std::mutex seen_mutex;
    std::vector<uint32_t> seen;
    cli_support::for_each_with_progress(items, [&](const uint32_t &item) {
        const std::lock_guard lock(seen_mutex);
        seen.push_back(item);
    }, execution);

    std::ranges::sort(seen);
    CHECK(seen == std::vector<uint32_t>{3, 5, 8, 13});
}
