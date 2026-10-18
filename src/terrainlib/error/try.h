#pragma once

#include <concepts>
#include <expected>
#include <optional>
#include <source_location>
#include <string>
#include <type_traits>
#include <utility>

#include "error/Error.h"
#include "error/ErrorCode.h"
#include "error/utils.h"
#include "macros.h"
#include "type_utils.h"

namespace alp::detail {

// A plain code ALP_TRY caught from a failed result, with where it was caught and the message to add; it becomes the error the enclosing function returns.
template <ErrorCode E, typename M>
struct PendingCode {
    E code;
    M message;
    std::source_location location;

    // Becomes any other type the code converts to, dropping the location.
    template <typename Target>
        requires(!is_specialization_of_v<Target, BasicError> && std::is_convertible_v<const E &, Target>)
    operator Target() const {
        // Declared for every message so that std::expected sees the conversion, and rejected on use so that the reason is readable.
        static_assert(std::same_as<M, std::nullopt_t>, "ALP_TRY_CONTEXT can only be called in a function returning an alp::Result, a plain std::expected has no place for the message");
        return this->code;
    }

    // Becomes an Error with its origin where the code was caught and the message, if any, as its first context.
    template <ErrorCode... Es>
        requires std::is_convertible_v<Error<E>, BasicError<Es...>>
    operator BasicError<Es...>() const {
        if constexpr (std::same_as<M, std::nullopt_t>) {
            return make_error(this->code, this->location);
        } else {
            return make_error(this->code, this->message, this->location);
        }
    }
};

// The error ALP_TRY passes on for the `error` of a failed expected:
// - alp::BasicError: the error with a new frame, holding `message` if there is one
// - alp::ErrorCode: a PendingCode, which becomes the caller's error
// - any other error: the error unchanged
template <typename E, typename M>
auto make_residual_error(E error, M message, const std::source_location location) {
    constexpr bool has_message = !std::same_as<M, std::nullopt_t>;
    if constexpr (is_specialization_of_v<E, BasicError>) {
        if constexpr (has_message) {
            return std::move(error).context(std::move(message), location);
        } else {
            return std::move(error).propagate(location);
        }
    } else if constexpr (ErrorCode<E>) {
        return PendingCode<E, M>{
            .code = error, 
            .message = std::move(message), 
            .location = location};
    } else {
        static_assert(!has_message, "ALP_TRY_CONTEXT needs an expression whose error is an alp::Error or an alp::ErrorCode, any other error has no place for the message");
        return error;
    }
}

// What ALP_TRY returns for a failed `result`:
// - an optional: nullopt
// - an expected: the error to pass on, as unexpected
template <typename R, typename M>
auto make_residual(R &&result, M message, const std::source_location location) {
    if constexpr (is_specialization_of_v<std::remove_cvref_t<R>, std::optional>) {
        static_assert(std::same_as<M, std::nullopt_t>, "ALP_TRY_CONTEXT needs an expression whose result is an std::expected, an std::optional has no place for the message");
        return std::nullopt;
    } else if constexpr (is_specialization_of_v<std::remove_cvref_t<R>, std::expected>) {
        return std::unexpected(make_residual_error(std::forward<R>(result).error(), std::move(message), location));
    } else {
        static_assert(false, "ALP_TRY needs an expression whose result is a std::expected or a std::optional");
    }
}

// The value held by a succeeded `result`, as a reference into it.
template <typename R>
decltype(auto) take_value(R &&result) {
    static_assert(!std::is_void_v<decltype(*result)>, "ALP_TRY with a binding needs an expression whose result holds a value");
    return *std::forward<R>(result);
}

} // namespace alp::detail

// ALP_TRY(expr) with its result variable and message spelled out; the message is nullopt for none.
#define ALP_TRY_IMPL_DISCARD(result, message, ...)                                                              \
    do {                                                                                                        \
        auto result = (__VA_ARGS__);                                                                            \
        if (!result) {                                                                                          \
            return ::alp::detail::make_residual(std::move(result), (message), std::source_location::current()); \
        }                                                                                                       \
    } while (false)

// ALP_TRY(target, expr) with its result variable and message spelled out; the message is nullopt for none.
#define ALP_TRY_IMPL_BIND(result, target, message, ...)                                                     \
    auto result = (__VA_ARGS__);                                                                            \
    if (!result) {                                                                                          \
        return ::alp::detail::make_residual(std::move(result), (message), std::source_location::current()); \
    }                                                                                                       \
    target = ::alp::detail::take_value(std::move(result))

// ALP_TRY(expr).
#define ALP_TRY_1(expr) ALP_TRY_IMPL_DISCARD(ALP_UNIQUE_IDENT(_alp_try_result_), ::std::nullopt, expr)

// ALP_TRY(target, expr).
#define ALP_TRY_2(target, expr) ALP_TRY_IMPL_BIND(ALP_UNIQUE_IDENT(_alp_try_result_), target, ::std::nullopt, expr)

// ALP_TRY_CONTEXT(expr, message).
#define ALP_TRY_CONTEXT_2(expr, message) ALP_TRY_IMPL_DISCARD(ALP_UNIQUE_IDENT(_alp_try_result_), message, expr)

// ALP_TRY_CONTEXT(target, expr, message).
#define ALP_TRY_CONTEXT_3(target, expr, message) ALP_TRY_IMPL_BIND(ALP_UNIQUE_IDENT(_alp_try_result_), target, message, expr)

// Returns from the enclosing function if `expr` failed; ALP_TRY(target, expr) otherwise binds its value to `target`.
// An expression with commas outside parentheses needs its own parentheses.
#define ALP_TRY(...) ALP_CAT_IDENT(ALP_TRY_, ALP_COUNT_ARGS(__VA_ARGS__))(__VA_ARGS__)

// ALP_TRY_CONTEXT(expr, message) or ALP_TRY_CONTEXT(target, expr, message).
#define ALP_TRY_CONTEXT(...) ALP_CAT_IDENT(ALP_TRY_CONTEXT_, ALP_COUNT_ARGS(__VA_ARGS__))(__VA_ARGS__)
