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
#include <ranges>
#include <vector>

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

// `x` with the value category of T and the constness of T added, as std::forward_like.
template <typename T, typename U>
[[nodiscard]]
constexpr like_t<T, U> forward_like(U &&x) noexcept {
    // TODO: switch back to std::forward_like once CI no longer builds with clang against libstdc++ 14.2
    return static_cast<like_t<T, U>>(x);
}

// Appends the elements of `range` to `vector`, as std::vector::append_range for a range whose begin and end have the same type.
template <typename T, typename Allocator, std::ranges::common_range Range>
    requires std::convertible_to<std::ranges::range_reference_t<Range>, T>
void append_range(std::vector<T, Allocator> &vector, Range &&range) {
    // TODO: switch back to std::vector::append_range once CI no longer builds against libstdc++ 14
    vector.insert(vector.end(), std::ranges::begin(range), std::ranges::end(range));
}

} // namespace std_compat
