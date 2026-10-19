#pragma once

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <string_view>
#include <typeinfo>
#include <memory>
#include <cstdlib>
#include <type_traits>

#ifndef _MSC_VER
    #include <cxxabi.h>
#endif

// modified from https://stackoverflow.com/questions/1055452/c-get-name-of-type-in-template/59522794#59522794
namespace detail {
template <typename T>
[[nodiscard]] constexpr std::string_view function_signature() {
#ifndef _MSC_VER
    return __PRETTY_FUNCTION__;
#else
    return __FUNCSIG__;
#endif
}

struct TypeNameFormat {
    std::size_t junk_leading = 0;
    std::size_t junk_total = 0;
};

inline constexpr TypeNameFormat type_name_format = [] {
    TypeNameFormat ret;
    std::string_view sample = function_signature<int>();
    ret.junk_leading = sample.find("int");
    ret.junk_total = sample.size() - 3;
    return ret;
}();
static_assert(type_name_format.junk_leading != std::size_t(-1), "Unable to determine the type name format on this compiler.");

template <typename T>
inline constexpr auto type_name_storage = [] {
    std::array<char, function_signature<T>().size() - type_name_format.junk_total + 1> ret{};
    std::copy_n(function_signature<T>().data() + type_name_format.junk_leading, ret.size() - 1, ret.data());
    ret.back() = '\0';
    return ret;
}();
}

template <typename T>
[[nodiscard]] inline constexpr std::string_view type_name() {
    return {detail::type_name_storage<T>.data(), detail::type_name_storage<T>.size() - 1};
}

template <typename T>
[[nodiscard]] inline constexpr const char *type_name_c() {
    return detail::type_name_storage<T>.data();
}

template <class T>
inline std::string type_name(const T &obj) {
#ifndef _MSC_VER
    int status = 0;
    std::unique_ptr<char, void (*)(void *)> res{
        abi::__cxa_demangle(typeid(obj).name(), nullptr, nullptr, &status),
        std::free};
    if (status == 0 && res)
        return res.get();
#endif
    return typeid(obj).name(); // fallback (may be mangled)
}

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
