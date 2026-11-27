#pragma once

#include <concepts>
#include <expected>
#include <functional>
#include <optional>
#include <source_location>
#include <string>
#include <type_traits>
#include <utility>

#include "error/Context.h"
#include "error/Error.h"
#include "error/ErrorCode.h"
#include "error/Result.h"
#include "optional_utils.h"
#include "type_utils.h"

namespace alp {

// Creates an error with `code` that originates at the caller.
template <ErrorCode E>
[[nodiscard]]
Error<E> make_error(E code, const std::source_location location = std::source_location::current()) {
    return Error<E>(std::move(code), location);
}

// Creates an error with `code` that originates at the caller, with `context` as its first context.
template <ErrorCode E>
[[nodiscard]]
Error<E> make_error(E code, Context auto &&context, const std::source_location location = std::source_location::current()) {
    return Error<E>(std::move(code), std::forward<decltype(context)>(context), location);
}

// Creates an error with `code` that originates at the caller, created from `inner`, or from an error with the code `inner` that originates there too.
template <ErrorCode E, typename F> requires(ErrorCode<F> || is_specialization_of_v<F, BasicError>)
[[nodiscard]]
Error<E> make_error(E code, F inner, const std::source_location location = std::source_location::current()) {
    if constexpr (ErrorCode<F>) {
        return Error<E>(std::move(code), Error<F>(std::move(inner), location), location);
    } else {
        return Error<E>(std::move(code), std::move(inner), location);
    }
}

// Creates an error with `code` that originates at the caller, created from `inner`, or from an error with the code `inner` that originates there too, with `context` added at the caller.
template <ErrorCode E, typename F> requires(ErrorCode<F> || is_specialization_of_v<F, BasicError>)
[[nodiscard]]
Error<E> make_error(E code, Context auto &&context, F inner, const std::source_location location = std::source_location::current()) {
    return make_error(std::move(code), std::move(inner), location)
        .context(std::forward<decltype(context)>(context), location);
}

// Creates an erased error from `message` that originates at the caller.
[[nodiscard]]
inline ErasedError make_error(std::string message, const std::source_location location = std::source_location::current()) {
    return ErasedError(std::move(message), location);
}

// Creates an unexpected holding a new error with `code` that originates at the caller.
template <ErrorCode E>
[[nodiscard]]
Unexpected<E> make_unexpected(E code, const std::source_location location = std::source_location::current()) {
    return std::unexpected(make_error(std::move(code), location));
}

// Creates an unexpected holding a new error with `code` that originates at the caller, with `context` as its first context.
template <ErrorCode E>
[[nodiscard]]
Unexpected<E> make_unexpected(E code, Context auto &&context, const std::source_location location = std::source_location::current()) {
    return std::unexpected(make_error(std::move(code), std::forward<decltype(context)>(context), location));
}

// Creates an unexpected holding a new erased error from `message` that originates at the caller.
[[nodiscard]]
inline Unexpected<> make_unexpected(std::string message, const std::source_location location = std::source_location::current()) {
    return std::unexpected(make_error(std::move(message), location));
}

// Adds a frame at the caller to `error` and wraps it in an unexpected.
template <ErrorCode... Es>
[[nodiscard]]
BasicUnexpected<BasicError<Es...>> propagate(BasicError<Es...> error, const std::source_location location = std::source_location::current()) {
    return std::unexpected(std::move(error).propagate(location));
}

// Adds a frame holding `context` to `result` if it failed.
template <typename T, ErrorCode... Es>
[[nodiscard]]
BasicResult<T, BasicError<Es...>> with_context(
    BasicResult<T, BasicError<Es...>> &&result,
    Context auto &&context,
    const std::source_location location = std::source_location::current()) {
    if (!result.has_value()) {
        return std::unexpected(std::move(result).error().context(std::forward<decltype(context)>(context), location));
    }
    return std::move(result);
}

// Runs `body` and, if it fails, adds a frame holding `context` at the caller.
template <std::invocable F>
[[nodiscard]]
std::invoke_result_t<F> run_with_context(Context auto &&context, F &&body, const std::source_location location = std::source_location::current()) {
    return alp::with_context(std::invoke(std::forward<F>(body)), std::forward<decltype(context)>(context), location);
}

// Turns `value` into a result that fails with `code` when it is empty.
template <typename T, ErrorCode E>
[[nodiscard]]
Result<T, E> to_result(std::optional<T> value, E code, const std::source_location location = std::source_location::current()) {
    return to_expected_or_else(std::move(value), [&] { return alp::make_error(std::move(code), location); });
}

// Turns `value` into a result that fails with `code` when it is empty, with `context` as the error's first context.
template <typename T, ErrorCode E>
[[nodiscard]]
Result<T, E> to_result(
    std::optional<T> value,
    E code,
    Context auto &&context,
    const std::source_location location = std::source_location::current()) {
    return to_expected_or_else(std::move(value), [&] { return alp::make_error(std::move(code), std::forward<decltype(context)>(context), location); });
}

// Turns `value` into a result that fails with an erased error from `message` when it is empty.
template <typename T>
[[nodiscard]]
ErasedResult<T> to_result(std::optional<T> value, std::string message, const std::source_location location = std::source_location::current()) {
    return to_expected_or_else(std::move(value), [&] { return alp::make_error(std::move(message), location); });
}

} // namespace alp
