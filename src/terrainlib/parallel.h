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

#pragma once

#include <atomic>
#include <concepts>
#include <expected>
#include <iterator>
#include <ranges>
#include <stop_token>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <libassert/assert.hpp>
#include <tbb/parallel_for_each.h>
#include <tbb/task_group.h>

#include "error/FallibleFn.h"
#include "error/Infallible.h"
#include "error/Result.h"
#include "error/log.h"
#include "error/utils.h"
#include "error/unwrap.h"
#include "stop.h"
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

// A function that takes an item of `Range` and a feeder for further items.
template <typename F, typename Range>
concept ItemAndFeederFn = std::invocable<F, const std::ranges::range_value_t<Range> &, Feeder<std::ranges::range_value_t<Range>> &>;

// A function that takes an item of `Range`.
template <typename F, typename Range>
concept ItemFn = std::invocable<F, const std::ranges::range_value_t<Range> &>;

// A function that takes an item of `Range` and a feeder for further items, and returns a result without a value.
template <typename F, typename Range>
concept FallibleItemAndFeederFn = ItemAndFeederFn<F, Range> && alp::FallibleFn<F, void, const std::ranges::range_value_t<Range> &, Feeder<std::ranges::range_value_t<Range>> &>;

// A function that takes an item of `Range` and returns a result without a value.
template <typename F, typename Range>
concept FallibleItemFn = ItemFn<F, Range> && alp::FallibleFn<F, void, const std::ranges::range_value_t<Range> &>;

namespace detail {

template <typename Range, typename F>
auto try_for_each_serial(Range &range, F &f, const std::stop_token &stop) {
    using Item = std::ranges::range_value_t<Range>;
    using Result = alp::Result<void, typename std::invoke_result_t<F &, const Item &, Feeder<Item> &>::error_type, Stopped>;

    std::vector<Item> added;
    Feeder<Item> feeder(added);
    for (const Item &item : range) {
        if (stop.stop_requested()) {
            return Result(alp::make_error(Stopped{}));
        }

        Result result = f(item, feeder);
        if (!result.has_value()) {
            return result;
        }
    }

    while (!added.empty()) {
        const std::vector<Item> batch = std::exchange(added, {});
        for (const Item &item : batch) {
            if (stop.stop_requested()) {
                return Result(alp::make_error(Stopped{}));
            }

            Result result = f(item, feeder);
            if (!result.has_value()) {
                return result;
            }
        }
    }

    return Result();
}

template <typename Range, typename F>
auto try_for_each_parallel(Range &range, F &f, const std::stop_token &stop) {
    using Item = std::ranges::range_value_t<Range>;
    using Result = alp::Result<void, typename std::invoke_result_t<F &, const Item &, Feeder<Item> &>::error_type, Stopped>;

    tbb::task_group_context context(tbb::task_group_context::isolated);
    const std::stop_callback cancel_on_stop(stop, [&] { context.cancel_group_execution(); });

    Result loop_result;
    tbb::parallel_for_each(std::ranges::begin(range), std::ranges::end(range), [&](const Item &item, tbb::feeder<Item> &sink) {
        // Cancelling the group doesnt stop an item whose task has already started.
        if (context.is_group_execution_cancelled()) {
            return;
        }

        Feeder<Item> feeder(sink);
        Result result = f(item, feeder);
        if (result.has_value()) {
            return;
        }

        // Exactly one of several concurrent cancel requests returns true, so only one body writes the result.
        if (context.cancel_group_execution()) {
            loop_result = std::move(result);
        } else {
            LOG_REPORT(result.error(), "processing an item of a parallel loop that had already failed");
        }
    }, context);

    if (loop_result.has_value() && stop.stop_requested()) {
        return Result(alp::make_error(Stopped{}));
    }
    return loop_result;
}

// Wraps `f` in a function that also takes a feeder and ignores it.
template <typename Range, typename F>
auto ignore_feeder(F &f) {
    using Item = std::ranges::range_value_t<Range>;
    return [&f](const Item &item, Feeder<Item> &) {
        return f(item);
    };
}

} // namespace detail

// Runs `f` on every item of `range`.
template <std::ranges::common_range Range, std::invocable<std::ranges::range_reference_t<Range>> F>
void for_each(Range &&range, F f, const Execution execution = Execution::Parallel) {
    switch (execution) {
    case Execution::Parallel:
        tbb::parallel_for_each(std::ranges::begin(range), std::ranges::end(range), f);
        return;
    case Execution::Serial:
        for (auto &&item : range) {
            f(item);
        }
        return;
    }
    UNREACHABLE("invalid execution", execution);
}

// Runs `f` on every item of `range`, and on every item `f` adds through its feeder, until a stop is requested through `stop`.
// Returns the first failure, or Stopped if the stop request came first, and skips the items that have not started by then.
template <std::ranges::common_range Range, FallibleItemAndFeederFn<Range> F>
[[nodiscard]]
auto try_for_each(Range &&range, F f, const std::stop_token &stop, const Execution execution = Execution::Parallel) {
    switch (execution) {
    case Execution::Parallel:
        return detail::try_for_each_parallel(range, f, stop);
    case Execution::Serial:
        return detail::try_for_each_serial(range, f, stop);
    }
    UNREACHABLE("invalid execution", execution);
}

// Runs `f` on every item of `range` until a stop is requested through `stop`.
// Returns the first failure, or Stopped if the stop request came first, and skips the items that have not started by then.
template <std::ranges::common_range Range, FallibleItemFn<Range> F>
[[nodiscard]]
auto try_for_each(Range &&range, F f, const std::stop_token &stop, const Execution execution = Execution::Parallel) {
    return try_for_each(range, detail::ignore_feeder<Range>(f), stop, execution);
}

// Runs `f` on every item of `range`, and on every item `f` adds through its feeder.
// Stops at the first failure, which it returns, and skips the items that have not started by then.
template <std::ranges::common_range Range, FallibleItemAndFeederFn<Range> F>
[[nodiscard]]
auto try_for_each(Range &&range, F f, const Execution execution = Execution::Parallel) {
    return alp::unwrap_partial<Stopped>(try_for_each(range, f, std::stop_token(), execution));
}

// Runs `f` on every item of `range`.
// Stops at the first failure, which it returns, and skips the items that have not started by then.
template <std::ranges::common_range Range, FallibleItemFn<Range> F>
[[nodiscard]]
auto try_for_each(Range &&range, F f, const Execution execution = Execution::Parallel) {
    return alp::unwrap_partial<Stopped>(try_for_each(range, f, std::stop_token(), execution));
}

// Runs `f` on every item of `range`, and on every item `f` adds through its feeder, until a stop is requested through `stop`.
// Skips the items that have not started by then.
template <std::ranges::common_range Range, ItemAndFeederFn<Range> F>
void for_each(Range &&range, F f, const std::stop_token &stop, const Execution execution = Execution::Parallel) {
    using Item = std::ranges::range_value_t<Range>;
    // The result can only hold Stopped, and a stopped loop has nothing more to report.
    std::ignore = try_for_each(range, [&](const Item &item, Feeder<Item> &feeder) -> alp::Result<void, alp::Infallible> {
        f(item, feeder);
        return {};
    }, stop, execution);
}

// Runs `f` on every item of `range`, and on every item `f` adds through its feeder.
template <std::ranges::common_range Range, ItemAndFeederFn<Range> F>
void for_each(Range &&range, F f, const Execution execution = Execution::Parallel) {
    for_each(range, f, std::stop_token(), execution);
}

// Runs `f` on every item of `range` until a stop is requested through `stop`.
// Skips the items that have not started by then.
template <std::ranges::common_range Range, ItemFn<Range> F>
void for_each(Range &&range, F f, const std::stop_token &stop, const Execution execution = Execution::Parallel) {
    for_each(range, detail::ignore_feeder<Range>(f), stop, execution);
}

} // namespace parallel
