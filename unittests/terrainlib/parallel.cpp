#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <fmt/format.h>

#include "../result_helpers.h"
#include "error/Error.h"
#include "error/Result.h"
#include "error/utils.h"
#include "parallel.h"

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

    const alp::Error<FailedItem> error = REQUIRE_ERROR(parallel::try_for_each(items, [&](const uint32_t &item) -> alp::Result<void, FailedItem> {
        if (item == 8) {
            return alp::make_unexpected(FailedItem{item});
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
    const alp::Error<FailedItem> error = REQUIRE_ERROR(parallel::try_for_each(items, [&](const uint32_t &item) -> alp::Result<void, FailedItem> {
        seen.push_back(item);
        if (item == 5 || item == 13) {
            return alp::make_unexpected(FailedItem{item});
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

    const alp::Error<FailedItem> error = REQUIRE_ERROR(parallel::try_for_each(items, [&](const uint32_t &item) -> alp::Result<void, FailedItem> {
        if (item == 5 || item == 13) {
            return alp::make_unexpected(FailedItem{item});
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

    const alp::Error<FailedItem> error = REQUIRE_ERROR(parallel::try_for_each(items, [&](const uint32_t &item, parallel::Feeder<uint32_t> &feeder) -> alp::Result<void, FailedItem> {
        if (item == 6) {
            return alp::make_unexpected(FailedItem{item});
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
