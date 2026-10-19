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
