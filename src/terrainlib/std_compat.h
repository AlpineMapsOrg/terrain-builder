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
#include <ranges>
#include <utility>
#include <vector>
#include <version>

#include <libassert/assert.hpp>

// Stand-ins for standard library facilities that a supported standard library lacks or that a supported compiler rejects.
namespace std_compat {

// like_t and forward_like mirror libstdc++ 15.2: https://github.com/gcc-mirror/gcc/blob/releases/gcc-15.2.0/libstdc++-v3/include/bits/move.h#L92-L126
namespace detail {

// T must be a reference and U an lvalue reference.
template <typename T, typename U>
struct like_impl;

template <typename T, typename U>
struct like_impl<T &, U &> {
    using type = U &;
};

template <typename T, typename U>
struct like_impl<const T &, U &> {
    using type = const U &;
};

template <typename T, typename U>
struct like_impl<T &&, U &> {
    using type = U &&;
};

template <typename T, typename U>
struct like_impl<const T &&, U &> {
    using type = const U &&;
};

} // namespace detail

// U with the constness of T added, as an lvalue reference if T is one and as an rvalue reference otherwise.
template <typename T, typename U>
using like_t = typename detail::like_impl<T &&, U &>::type;

namespace detail {

// Stand-in for std::forward_like.
template <typename T, typename U>
[[nodiscard]]
constexpr like_t<T, U> forward_like_fallback(U &&x) noexcept {
    return static_cast<like_t<T, U>>(x);
}

} // namespace detail

// Stand-in for std::forward_like.
template <typename T, typename U>
[[nodiscard]]
constexpr like_t<T, U> forward_like(U &&x) noexcept {
    // TODO: switch back to std::forward_like once CI no longer builds with clang against libstdc++ 14.2
#if defined(__cpp_lib_forward_like) && !(defined(__clang__) && defined(_GLIBCXX_RELEASE) && _GLIBCXX_RELEASE == 14)
    return std::forward_like<T>(std::forward<U>(x));
#else
    return detail::forward_like_fallback<T>(std::forward<U>(x));
#endif
}

namespace detail {

// Asserts that `range` and `vector` do not overlap, if `range` is contiguous and holds elements of type T.
template <typename T, typename Allocator, std::ranges::input_range Range>
void assert_disjoint(const std::vector<T, Allocator> &vector, Range &&range) {
    if constexpr (std::ranges::contiguous_range<Range> && std::ranges::sized_range<Range> && std::same_as<std::ranges::range_value_t<Range>, T>) {
        const T *const range_first = std::ranges::data(range);
        const T *const range_last = range_first + std::ranges::size(range);
        const T *const vector_first = vector.data();
        const T *const vector_last = vector_first + vector.size();
        DEBUG_ASSERT(range_first == range_last || !std::less{}(range_first, vector_last) || !std::less{}(vector_first, range_last));
    }
}

// Stand-in for std::vector::append_range.
// `range` and `vector` must not overlap.
template <typename T, typename Allocator, std::ranges::common_range Range>
    requires std::convertible_to<std::ranges::range_reference_t<Range>, T>
void append_range_fallback(std::vector<T, Allocator> &vector, Range &&range) {
    vector.insert(vector.end(), std::ranges::begin(range), std::ranges::end(range));
}

} // namespace detail

// Stand-in for std::vector::append_range.
// `range` and `vector` must not overlap.
template <typename T, typename Allocator, std::ranges::common_range Range>
    requires std::convertible_to<std::ranges::range_reference_t<Range>, T>
void append_range(std::vector<T, Allocator> &vector, Range &&range) {
    // The fallback does not support overlap.
    detail::assert_disjoint(vector, range);

    // TODO: switch back to std::vector::append_range once CI no longer builds against libstdc++ 14
#if defined(__cpp_lib_containers_ranges) && __cpp_lib_containers_ranges >= 202202L
    vector.append_range(std::forward<Range>(range));
#else
    detail::append_range_fallback(vector, std::forward<Range>(range));
#endif
}

} // namespace std_compat
