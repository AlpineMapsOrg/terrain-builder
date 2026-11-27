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

#include <concepts>
#include <functional>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>

#include <fmt/format.h>

namespace alp {

// A context, or a callable that builds one.
template <typename M>
concept Context = std::convertible_to<M, std::string>
    || (std::invocable<M> && std::convertible_to<std::invoke_result_t<M>, std::string>);

namespace detail {

// `context` as text, built now if it is a callable.
std::string resolve_message(Context auto &&context) {
    if constexpr (std::convertible_to<decltype(context), std::string>) {
        return std::string(std::forward<decltype(context)>(context));
    } else {
        return std::string(std::invoke(std::forward<decltype(context)>(context)));
    }
}

} // namespace detail

// A context that formats `format` with `args` when it is asked for.
// Holds lvalue arguments by reference and rvalue arguments by value, so it must not outlive the lvalues.
// TODO: rename to make_lazy_context
template <typename... Args>
[[nodiscard]]
Context auto make_context(const fmt::format_string<Args...> format, Args &&...args) {
    return [format = format.get(), arguments = std::tuple<Args...>(std::forward<Args>(args)...)]() {
        // The format string was checked against the arguments where the context was made.
        return std::apply([&](const auto &...values) { return fmt::vformat(format, fmt::make_format_args(values...)); }, arguments);
    };
}

} // namespace alp
