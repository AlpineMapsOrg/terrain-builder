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

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <iterator>
#include <optional>
#include <span>
#include <utility>

// The elements before and after the element of `span` at `index`, or null when `span` does not reach it; the element at `index` is in neither half.
template <typename T>
std::optional<std::pair<std::span<const T>, std::span<const T>>> split_once(const std::span<const T> span, const size_t index) {
    if (index >= span.size()) {
        return std::nullopt;
    }
    return std::pair(span.subspan(0, index), span.subspan(index + 1));
}

// The elements before and after the first element of `span` satisfying `predicate`, or null when none does; the matched element is in neither half.
template <typename T, std::predicate<const T &> F>
std::optional<std::pair<std::span<const T>, std::span<const T>>> split_once(const std::span<const T> span, F predicate) {
    const auto it = std::ranges::find_if(span, predicate);
    if (it == span.end()) {
        return std::nullopt;
    }
    const size_t index = std::distance(span.begin(), it);
    return split_once(span, index);
}

// The first element of `span` and the elements after it, or null when `span` is empty.
template <typename T>
std::optional<std::pair<const T *, std::span<const T>>> split_first(const std::span<const T> span) {
    if (span.empty()) {
        return std::nullopt;
    }
    return std::pair(&span.front(), span.subspan(1));
}
