#pragma once

#include <algorithm>
#include <compare>
#include <ranges>
#include <tuple>
#include <type_traits>

#include <glm/glm.hpp>
#include <radix/geometry.h>

namespace geometry {

// Orders scalars in a total order, so NaN and signed zeros have a place.
template <typename T>
    requires std::is_arithmetic_v<T>
[[nodiscard]] std::strong_ordering compare_strong(const T &left, const T &right) {
    return std::strong_order(left, right);
}

// Orders vectors by their components, first to last.
template <glm::length_t N, typename T>
[[nodiscard]] std::strong_ordering compare_strong(const glm::vec<N, T> &left, const glm::vec<N, T> &right) {
    for (glm::length_t i = 0; i < N; i++) {
        const std::strong_ordering order = compare_strong(left[i], right[i]);
        if (order != std::strong_ordering::equal) {
            return order;
        }
    }
    return std::strong_ordering::equal;
}

// Orders boxes by their minimum corner, then by their maximum corner.
struct MinThenMax {
    template <glm::length_t N, typename T>
    [[nodiscard]] static std::strong_ordering compare(const radix::geometry::Aabb<N, T> &left, const radix::geometry::Aabb<N, T> &right) {
        const std::strong_ordering min_order = compare_strong(left.min, right.min);
        if (min_order != std::strong_ordering::equal) {
            return min_order;
        }
        return compare_strong(left.max, right.max);
    }
};

// Orders boxes by `Order`.
// Without one, the order is fixed but unspecified.
template <typename Order = MinThenMax, glm::length_t N, typename T>
[[nodiscard]] std::strong_ordering compare_strong(const radix::geometry::Aabb<N, T> &left, const radix::geometry::Aabb<N, T> &right) {
    return Order::compare(left, right);
}

// Orders ranges by their elements, first to last.
// A range that is the start of the other comes first.
template <std::ranges::forward_range Range>
[[nodiscard]] std::strong_ordering compare_strong(const Range &left, const Range &right) {
    return std::lexicographical_compare_three_way(
        std::ranges::begin(left), std::ranges::end(left),
        std::ranges::begin(right), std::ranges::end(right),
        [](const auto &a, const auto &b) { return compare_strong(a, b); });
}

// A value that compares in the strong order of `compare_strong`.
template <typename T>
struct Strong {
    T value;

    std::strong_ordering operator<=>(const Strong &other) const {
        return compare_strong(this->value, other.value);
    }

    bool operator==(const Strong &other) const {
        return compare_strong(this->value, other.value) == std::strong_ordering::equal;
    }
};

// The given values as a tuple that compares in strong order.
// The tuple refers to the values and must not outlive them.
template <typename... Ts>
[[nodiscard]] std::tuple<Strong<const Ts &>...> tie_strong(const Ts &...values) {
    return std::tuple<Strong<const Ts &>...>(Strong<const Ts &>{values}...);
}

}
