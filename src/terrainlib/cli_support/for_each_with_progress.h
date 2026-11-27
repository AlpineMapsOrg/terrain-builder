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

#pragma once

#include <ranges>
#include <stop_token>
#include <thread>
#include <utility>

#include "cli_support/ProgressIndicator.h"
#include "parallel.h"

namespace cli_support {

// Runs `f` on every item of `range` until a stop is requested through `stop`, showing a progress bar that counts each item `f` finishes.
// Returns the first failure, or Stopped if the stop request came first.
template <std::ranges::sized_range Range, parallel::FallibleItemFn<Range> F>
    requires std::ranges::common_range<Range>
[[nodiscard]]
auto try_for_each_with_progress(Range &&range, F f, const std::stop_token &stop, const parallel::Execution execution = parallel::Execution::Parallel) {
    using Reference = std::ranges::range_reference_t<Range>;
    ProgressIndicator progress(std::ranges::size(range));
    const std::jthread monitor = progress.start_monitoring();
    return parallel::try_for_each(range, [&](Reference item) {
        auto result = f(std::forward<Reference>(item));
        progress.task_finished();
        return result;
    }, stop, execution);
    // The destructor of `monitor` requests a stop and joins it here, which finishes the bar.
}

// Runs `f` on every item of `range`, showing a progress bar that counts each item `f` finishes.
template <std::ranges::sized_range Range, parallel::ItemFn<Range> F>
    requires std::ranges::common_range<Range>
void for_each_with_progress(Range &&range, F f, const parallel::Execution execution = parallel::Execution::Parallel) {
    using Reference = std::ranges::range_reference_t<Range>;
    ProgressIndicator progress(std::ranges::size(range));
    const std::jthread monitor = progress.start_monitoring();
    parallel::for_each(range, [&](Reference item) {
        f(std::forward<Reference>(item));
        progress.task_finished();
    }, execution);
    // The destructor of `monitor` requests a stop and joins it here, which finishes the bar.
}

} // namespace cli_support
