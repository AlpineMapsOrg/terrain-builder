/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2025 Martin Braunsperger
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
#include <type_traits>
#include <utility>

template <class>
inline constexpr bool always_false_v = false;


template <typename T, template <typename...> class Tmpl>
struct is_specialization_of : std::false_type {};

template <template <typename...> class Tmpl, typename... Args>
struct is_specialization_of<Tmpl<Args...>, Tmpl> : std::true_type {};

template <typename T, template <typename...> class Tmpl>
inline constexpr bool is_specialization_of_v = is_specialization_of<T, Tmpl>::value;

// Whether T is one of Ts.
template <typename T, typename... Ts>
concept OneOf = (std::same_as<T, Ts> || ...);

// Whether a T converts to bool, and so does its negation.
// The standard's exposition-only boolean-testable, [concept.booleantestable].
template <typename T>
concept BooleanTestable = std::convertible_to<T, bool> && requires(T &&t) {
    { !std::forward<T>(t) } -> std::convertible_to<bool>;
};

// Whether values of T and U can be compared with == and != in either order.
// The standard's exposition-only weakly-equality-comparable-with, [concept.equalitycomparable].
template <typename T, typename U>
concept WeaklyEqualityComparableWith = requires(const std::remove_reference_t<T> &t, const std::remove_reference_t<U> &u) {
    { t == u } -> BooleanTestable;
    { t != u } -> BooleanTestable;
    { u == t } -> BooleanTestable;
    { u != t } -> BooleanTestable;
};

namespace detail {

// `Accumulated`, a specialization of a template, with Ts appended; an argument that is itself a specialization of that template is replaced by its arguments.
template <typename Accumulated, typename... Ts>
struct flatten_into;

template <template <typename...> class Tmpl, typename... As>
struct flatten_into<Tmpl<As...>> {
    using type = Tmpl<As...>;
};

template <template <typename...> class Tmpl, typename... As, typename T, typename... Ts>
struct flatten_into<Tmpl<As...>, T, Ts...> : flatten_into<Tmpl<As..., T>, Ts...> {};

template <template <typename...> class Tmpl, typename... As, typename... Bs, typename... Ts>
struct flatten_into<Tmpl<As...>, Tmpl<Bs...>, Ts...> : flatten_into<Tmpl<As...>, Bs..., Ts...> {};

template <typename T>
struct flatten;

template <template <typename...> class Tmpl, typename... Ts>
struct flatten<Tmpl<Ts...>> : flatten_into<Tmpl<>, Ts...> {};

// `Accumulated`, a specialization of a template, with each of Ts appended that it does not hold yet.
template <typename Accumulated, typename... Ts>
struct unique_into;

template <template <typename...> class Tmpl, typename... As>
struct unique_into<Tmpl<As...>> {
    using type = Tmpl<As...>;
};

template <template <typename...> class Tmpl, typename... As, typename T, typename... Ts>
struct unique_into<Tmpl<As...>, T, Ts...>
    : unique_into<std::conditional_t<OneOf<T, As...>, Tmpl<As...>, Tmpl<As..., T>>, Ts...> {};

template <typename T>
struct unique;

template <template <typename...> class Tmpl, typename... Ts>
struct unique<Tmpl<Ts...>> : unique_into<Tmpl<>, Ts...> {};

} // namespace detail

// T, a specialization of a template, with each argument that is itself a specialization of that template replaced by its arguments, at any depth.
template <typename T>
using flatten_t = typename detail::flatten<T>::type;

// T, a specialization of a template, with each argument kept once, in the order it first appears.
template <typename T>
using unique_t = typename detail::unique<T>::type;

// Whether every argument of the specialization T is among the arguments of the specialization U.
template <typename T, typename U>
inline constexpr bool is_subset_v = false;

template <template <typename...> class TTmpl, typename... Ts, template <typename...> class UTmpl, typename... Us>
inline constexpr bool is_subset_v<TTmpl<Ts...>, UTmpl<Us...>> = (OneOf<Ts, Us...> && ...);

namespace detail {

template <typename T, typename U>
struct difference;

template <template <typename...> class TTmpl, typename... Ts, template <typename...> class UTmpl, typename... Us>
struct difference<TTmpl<Ts...>, UTmpl<Us...>> {
    using type = flatten_t<TTmpl<std::conditional_t<OneOf<Ts, Us...>, TTmpl<>, TTmpl<Ts>>...>>;
};

} // namespace detail

// T, a specialization of a template, without the arguments that are among the arguments of the specialization U.
template <typename T, typename U>
using difference_t = typename detail::difference<T, U>::type;
