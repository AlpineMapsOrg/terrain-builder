#pragma once

#include <concepts>
#include <iterator>
#include <ranges>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <libassert/assert.hpp>
#include <tbb/parallel_for_each.h>
#include <tbb/task_group.h>

#include "error/FallibleFn.h"
#include "error/log.h"
#include "variant_utils.h"

namespace parallel {

// Whether a loop spreads its items over threads or runs them in order on the calling thread.
enum class Execution {
    Parallel,
    Serial,
};

// Takes the items a running body adds to the loop it is part of.
template <typename Item>
class Feeder {
public:
    explicit Feeder(tbb::feeder<Item> &sink) : _sink(&sink) {}
    explicit Feeder(std::vector<Item> &sink) : _sink(&sink) {}

    void add(Item item) {
        match(this->_sink,
            [&](tbb::feeder<Item> *sink) {
                sink->add(std::move(item));
            },
            [&](std::vector<Item> *sink) {
                sink->push_back(std::move(item));
            });
    }

private:
    std::variant<tbb::feeder<Item> *, std::vector<Item> *> _sink;
};

namespace detail {

template <typename Range, typename Body>
auto try_for_each_serial(Range &range, Body &body) {
    using Item = std::ranges::range_value_t<Range>;
    using Result = std::invoke_result_t<Body &, const Item &, Feeder<Item> &>;

    std::vector<Item> added;
    Feeder<Item> feeder(added);
    for (const Item &item : range) {
        Result result = body(item, feeder);
        if (!result.has_value()) {
            return result;
        }
    }
    while (!added.empty()) {
        const std::vector<Item> batch = std::exchange(added, {});
        for (const Item &item : batch) {
            Result result = body(item, feeder);
            if (!result.has_value()) {
                return result;
            }
        }
    }
    return Result();
}

template <typename Range, typename Body>
auto try_for_each_parallel(Range &range, Body &body) {
    using Item = std::ranges::range_value_t<Range>;
    using Result = std::invoke_result_t<Body &, const Item &, Feeder<Item> &>;

    // A bound context would also be cancelled from outside, and then no failing body could claim the cancellation.
    tbb::task_group_context context(tbb::task_group_context::isolated);
    Result loop_result;
    tbb::parallel_for_each(std::ranges::begin(range), std::ranges::end(range), [&](const Item &item, tbb::feeder<Item> &sink) {
        Feeder<Item> feeder(sink);
        Result result = body(item, feeder);
        if (result.has_value()) {
            return;
        }

        // Exactly one of several concurrent cancel requests returns true, so only one body writes the result.
        if (context.cancel_group_execution()) {
            loop_result = std::move(result);
        } else {
            LOG_UNEXPECTED(result.error(), "processing an item of a loop that had already failed");
        }
    }, context);
    return loop_result;
}

} // namespace detail

// Runs `body` on every item of `range`.
template <std::ranges::forward_range Range, std::invocable<std::ranges::range_reference_t<Range>> Body>
void for_each(Range &&range, Body body, const Execution execution = Execution::Parallel) {
    switch (execution) {
    case Execution::Parallel:
        tbb::parallel_for_each(std::ranges::begin(range), std::ranges::end(range), body);
        return;
    case Execution::Serial:
        for (auto &&item : range) {
            body(item);
        }
        return;
    }
    UNREACHABLE();
}

// Runs `body` on every item of `range`, and on every item a body adds through its feeder.
// Stops at the first failure, which it returns, and skips the items that have not started by then.
template <std::ranges::forward_range Range, alp::FallibleFn<void, const std::ranges::range_value_t<Range> &, Feeder<std::ranges::range_value_t<Range>> &> Body>
[[nodiscard]]
auto try_for_each(Range &&range, Body body, const Execution execution = Execution::Parallel) {
    switch (execution) {
    case Execution::Parallel:
        return detail::try_for_each_parallel(range, body);
    case Execution::Serial:
        return detail::try_for_each_serial(range, body);
    }
    UNREACHABLE();
}

// Runs `body` on every item of `range`.
// Stops at the first failure, which it returns, and skips the items that have not started by then.
template <std::ranges::forward_range Range, alp::FallibleFn<void, const std::ranges::range_value_t<Range> &> Body>
[[nodiscard]]
auto try_for_each(Range &&range, Body body, const Execution execution = Execution::Parallel) {
    using Item = std::ranges::range_value_t<Range>;
    return try_for_each(range, [&](const Item &item, Feeder<Item> &) {
        return body(item);
    }, execution);
}

} // namespace parallel
