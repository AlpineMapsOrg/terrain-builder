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
#include "error/Error.h"
#include "error/Result.h"
#include "error/utils.h"
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

TEST_CASE("for_each runs the body once for every item", "[parallel]") {
    const parallel::Execution execution = GENERATE(parallel::Execution::Parallel, parallel::Execution::Serial);
    const std::vector<uint32_t> items = {3, 5, 8, 13};

    // Catch2 assertions are not thread safe, so the body only records what it saw.
    std::mutex seen_mutex;
    std::vector<uint32_t> seen;
    parallel::for_each(items, [&](const uint32_t &item) {
        const std::lock_guard lock(seen_mutex);
        seen.push_back(item);
    }, execution);

    std::ranges::sort(seen);
    CHECK(seen == std::vector<uint32_t>{3, 5, 8, 13});
}

// Every item below 8 adds its two children in a binary heap numbering, so the root 1 reaches 1 to 15.
TEST_CASE("for_each runs the items a body adds through its feeder", "[parallel]") {
    const parallel::Execution execution = GENERATE(parallel::Execution::Parallel, parallel::Execution::Serial);
    const std::vector<uint32_t> items = {1};

    std::mutex seen_mutex;
    std::vector<uint32_t> seen;
    parallel::for_each(items, [&](const uint32_t &item, parallel::Feeder<uint32_t> &feeder) {
        if (item < 8) {
            feeder.add(2 * item);
            feeder.add(2 * item + 1);
        }
        const std::lock_guard lock(seen_mutex);
        seen.push_back(item);
    }, execution);

    std::ranges::sort(seen);
    CHECK(seen == std::vector<uint32_t>{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15});
}

TEST_CASE("for_each with a stop token starts no item after a body requested the stop in a serial run", "[parallel]") {
    const std::vector<uint32_t> items = {3, 5, 8, 13};
    std::stop_source stop;

    std::vector<uint32_t> seen;
    parallel::for_each(items, [&](const uint32_t &item) {
        seen.push_back(item);
        if (item == 5) {
            stop.request_stop();
        }
    }, stop.get_token(), parallel::Execution::Serial);

    CHECK(seen == std::vector<uint32_t>{3, 5});
}

TEST_CASE("for_each with a stop token starts no item that a body adds after it requested the stop", "[parallel]") {
    const parallel::Execution execution = GENERATE(parallel::Execution::Parallel, parallel::Execution::Serial);
    const std::vector<uint32_t> items = {1};
    std::stop_source stop;

    std::mutex seen_mutex;
    std::vector<uint32_t> seen;
    parallel::for_each(items, [&](const uint32_t &item, parallel::Feeder<uint32_t> &feeder) {
        if (item == 1) {
            stop.request_stop();
            feeder.add(2);
            feeder.add(3);
        }
        const std::lock_guard lock(seen_mutex);
        seen.push_back(item);
    }, stop.get_token(), execution);

    CHECK(seen == std::vector<uint32_t>{1});
}

TEST_CASE("try_for_each runs the body once for every item", "[parallel]") {
    const parallel::Execution execution = GENERATE(parallel::Execution::Parallel, parallel::Execution::Serial);
    const std::vector<uint32_t> items = {3, 5, 8, 13};

    // Catch2 assertions are not thread safe, so the body only records what it saw.
    std::mutex seen_mutex;
    std::vector<uint32_t> seen;
    REQUIRE_VALUE(parallel::try_for_each(items, [&](const uint32_t &item) -> alp::Result<void, FailedItem> {
        const std::lock_guard lock(seen_mutex);
        seen.push_back(item);
        return {};
    }, execution));

    std::ranges::sort(seen);
    CHECK(seen == std::vector<uint32_t>{3, 5, 8, 13});
}

TEST_CASE("try_for_each succeeds for an empty range without running the body", "[parallel]") {
    const parallel::Execution execution = GENERATE(parallel::Execution::Parallel, parallel::Execution::Serial);
    const std::vector<uint32_t> items;

    std::atomic<bool> ran = false;
    REQUIRE_VALUE(parallel::try_for_each(items, [&](const uint32_t &) -> alp::Result<void, FailedItem> {
        ran = true;
        return {};
    }, execution));

    CHECK_FALSE(ran);
}

TEST_CASE("try_for_each returns the error of the body that failed", "[parallel]") {
    const parallel::Execution execution = GENERATE(parallel::Execution::Parallel, parallel::Execution::Serial);
    const std::vector<uint32_t> items = {3, 5, 8, 13};

    const auto error = REQUIRE_ERROR(parallel::try_for_each(items, [&](const uint32_t &item) -> alp::Result<void, FailedItem> {
        if (item == 8) {
            return alp::make_error(FailedItem{item});
        }
        return {};
    }, execution));

    const FailedItem *failed = error.get_if<FailedItem>();
    REQUIRE(failed != nullptr);
    CHECK(failed->item == 8);
}

TEST_CASE("try_for_each runs no item after the first failure of a serial run", "[parallel]") {
    const std::vector<uint32_t> items = {3, 5, 8, 13};

    std::vector<uint32_t> seen;
    const auto error = REQUIRE_ERROR(parallel::try_for_each(items, [&](const uint32_t &item) -> alp::Result<void, FailedItem> {
        seen.push_back(item);
        if (item == 5 || item == 13) {
            return alp::make_error(FailedItem{item});
        }
        return {};
    }, parallel::Execution::Serial));

    CHECK(seen == std::vector<uint32_t>{3, 5});
    const FailedItem *failed = error.get_if<FailedItem>();
    REQUIRE(failed != nullptr);
    CHECK(failed->item == 5);
}

TEST_CASE("try_for_each returns one of the errors when several bodies of a parallel run fail", "[parallel]") {
    const std::vector<uint32_t> items = {3, 5, 8, 13};

    const auto error = REQUIRE_ERROR(parallel::try_for_each(items, [&](const uint32_t &item) -> alp::Result<void, FailedItem> {
        if (item == 5 || item == 13) {
            return alp::make_error(FailedItem{item});
        }
        return {};
    }, parallel::Execution::Parallel));

    const FailedItem *failed = error.get_if<FailedItem>();
    REQUIRE(failed != nullptr);
    const bool is_a_failed_item = failed->item == 5 || failed->item == 13;
    CHECK(is_a_failed_item);
}

// Every item below 8 adds its two children in a binary heap numbering, so the root 1 reaches 1 to 15.
TEST_CASE("try_for_each runs the items a body adds through its feeder", "[parallel]") {
    const parallel::Execution execution = GENERATE(parallel::Execution::Parallel, parallel::Execution::Serial);
    const std::vector<uint32_t> items = {1};

    std::mutex seen_mutex;
    std::vector<uint32_t> seen;
    REQUIRE_VALUE(parallel::try_for_each(items, [&](const uint32_t &item, parallel::Feeder<uint32_t> &feeder) -> alp::Result<void, FailedItem> {
        if (item < 8) {
            feeder.add(2 * item);
            feeder.add(2 * item + 1);
        }
        const std::lock_guard lock(seen_mutex);
        seen.push_back(item);
        return {};
    }, execution));

    std::ranges::sort(seen);
    CHECK(seen == std::vector<uint32_t>{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15});
}

TEST_CASE("try_for_each returns the error of an item that was added through the feeder", "[parallel]") {
    const parallel::Execution execution = GENERATE(parallel::Execution::Parallel, parallel::Execution::Serial);
    const std::vector<uint32_t> items = {1};

    const auto error = REQUIRE_ERROR(parallel::try_for_each(items, [&](const uint32_t &item, parallel::Feeder<uint32_t> &feeder) -> alp::Result<void, FailedItem> {
        if (item == 6) {
            return alp::make_error(FailedItem{item});
        }
        if (item < 8) {
            feeder.add(2 * item);
            feeder.add(2 * item + 1);
        }
        return {};
    }, execution));

    const FailedItem *failed = error.get_if<FailedItem>();
    REQUIRE(failed != nullptr);
    CHECK(failed->item == 6);
}

TEST_CASE("try_for_each with a stop token runs no body once the stop was requested before the loop", "[parallel]") {
    const parallel::Execution execution = GENERATE(parallel::Execution::Parallel, parallel::Execution::Serial);
    const std::vector<uint32_t> items = {3, 5, 8, 13};
    std::stop_source stop;
    stop.request_stop();

    std::atomic<bool> ran = false;
    const auto error = REQUIRE_ERROR(parallel::try_for_each(items, [&](const uint32_t &) -> alp::Result<void, FailedItem> {
        ran = true;
        return {};
    }, stop.get_token(), execution));

    CHECK(error.holds<Stopped>());
    CHECK_FALSE(ran);
}

TEST_CASE("try_for_each with a stop token returns Stopped once a body requests the stop", "[parallel]") {
    const parallel::Execution execution = GENERATE(parallel::Execution::Parallel, parallel::Execution::Serial);
    const std::vector<uint32_t> items = {3, 5, 8, 13};
    std::stop_source stop;

    const auto error = REQUIRE_ERROR(parallel::try_for_each(items, [&](const uint32_t &item) -> alp::Result<void, FailedItem> {
        if (item == 5) {
            stop.request_stop();
        }
        return {};
    }, stop.get_token(), execution));

    CHECK(error.holds<Stopped>());
}

TEST_CASE("try_for_each with a stop token starts no item after a body requested the stop in a serial run", "[parallel]") {
    const std::vector<uint32_t> items = {3, 5, 8, 13};
    std::stop_source stop;

    std::vector<uint32_t> seen;
    const auto error = REQUIRE_ERROR(parallel::try_for_each(items, [&](const uint32_t &item) -> alp::Result<void, FailedItem> {
        seen.push_back(item);
        if (item == 5) {
            stop.request_stop();
        }
        return {};
    }, stop.get_token(), parallel::Execution::Serial));

    CHECK(error.holds<Stopped>());
    CHECK(seen == std::vector<uint32_t>{3, 5});
}

TEST_CASE("try_for_each with a stop token returns the error of the body that failed", "[parallel]") {
    const parallel::Execution execution = GENERATE(parallel::Execution::Parallel, parallel::Execution::Serial);
    const std::vector<uint32_t> items = {3, 5, 8, 13};
    const std::stop_source stop;

    const auto error = REQUIRE_ERROR(parallel::try_for_each(items, [&](const uint32_t &item) -> alp::Result<void, FailedItem> {
        if (item == 8) {
            return alp::make_error(FailedItem{item});
        }
        return {};
    }, stop.get_token(), execution));

    const FailedItem *failed = error.get_if<FailedItem>();
    REQUIRE(failed != nullptr);
    CHECK(failed->item == 8);
}

TEST_CASE("try_for_each with a stop token starts no item that a body adds after it requested the stop", "[parallel]") {
    const parallel::Execution execution = GENERATE(parallel::Execution::Parallel, parallel::Execution::Serial);
    const std::vector<uint32_t> items = {1};
    std::stop_source stop;

    std::mutex seen_mutex;
    std::vector<uint32_t> seen;
    const auto error = REQUIRE_ERROR(parallel::try_for_each(items, [&](const uint32_t &item, parallel::Feeder<uint32_t> &feeder) -> alp::Result<void, FailedItem> {
        if (item == 1) {
            stop.request_stop();
            feeder.add(2);
            feeder.add(3);
        }
        const std::lock_guard lock(seen_mutex);
        seen.push_back(item);
        return {};
    }, stop.get_token(), execution));

    CHECK(error.holds<Stopped>());
    CHECK(seen == std::vector<uint32_t>{1});
}
