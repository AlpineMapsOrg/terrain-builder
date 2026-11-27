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
#include <expected>
#include <functional>
#include <type_traits>
#include <utility>

namespace alp {

// The value of `result`, or the one `f` makes from its error.
template <typename T, typename E, std::invocable<E> F>
    requires std::convertible_to<std::invoke_result_t<F, E>, T>
[[nodiscard]]
T value_or_else(std::expected<T, E> result, F &&f) {
    if (result.has_value()) {
        return *std::move(result);
    }
    return std::invoke(std::forward<F>(f), std::move(result).error());
}

} // namespace alp
