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
template <typename... Args>
[[nodiscard]]
Context auto make_context(const fmt::format_string<Args...> format, Args &&...args) {
    return [format = format.get(), arguments = std::tuple<Args...>(std::forward<Args>(args)...)]() {
        // The format string was checked against the arguments where the context was made.
        return std::apply([&](const auto &...values) { return fmt::vformat(format, fmt::make_format_args(values...)); }, arguments);
    };
}

} // namespace alp
