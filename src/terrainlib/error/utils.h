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

// Creates an error with `code` that originates at the caller, created from `inner`.
template <ErrorCode E, ErrorCode... Fs>
[[nodiscard]]
Error<E> make_error(E code, BasicError<Fs...> inner, const std::source_location location = std::source_location::current()) {
    return make_error(std::move(code), location).with_source(std::move(inner));
}

// Creates an error with `code` that originates at the caller, created from an error with the code `inner` that originates there too.
template <ErrorCode E, ErrorCode F>
[[nodiscard]]
Error<E> make_error(E code, F inner, const std::source_location location = std::source_location::current()) {
    return make_error(std::move(code), make_error(std::move(inner), location), location);
}

// Creates an error with `code` that originates at the caller, with `context` as its first context, created from `inner`.
template <ErrorCode E, ErrorCode... Fs>
[[nodiscard]]
Error<E> make_error(E code, Context auto &&context, BasicError<Fs...> inner, const std::source_location location = std::source_location::current()) {
    return make_error(std::move(code), std::forward<decltype(context)>(context), location).with_source(std::move(inner));
}

// Creates an error with `code` that originates at the caller, with `context` as its first context, created from an error with the code `inner` that originates there too.
template <ErrorCode E, ErrorCode F>
[[nodiscard]]
Error<E> make_error(E code, Context auto &&context, F inner, const std::source_location location = std::source_location::current()) {
    return make_error(std::move(code), std::forward<decltype(context)>(context), make_error(std::move(inner), location), location);
}

// Creates an error with `code` that originates at the caller, created from `inner` if there is one.
template <ErrorCode E, typename F> requires(ErrorCode<F> || is_specialization_of_v<F, BasicError>)
[[nodiscard]]
Error<E> make_error(E code, std::optional<F> inner, const std::source_location location = std::source_location::current()) {
    if (inner) {
        return make_error(std::move(code), std::move(*inner), location);
    } else {
        return make_error(std::move(code), location);
    }
}

// Creates an error with `code` that originates at the caller, created from `inner` if there is one, with `context` added at the caller.
template <ErrorCode E, typename F> requires(ErrorCode<F> || is_specialization_of_v<F, BasicError>)
[[nodiscard]]
Error<E> make_error(E code, Context auto &&context, std::optional<F> inner, const std::source_location location = std::source_location::current()) {
    if (inner) {
        return make_error(std::move(code), std::forward<decltype(context)>(context), std::move(*inner), location);
    } else {
        return make_error(std::move(code), std::forward<decltype(context)>(context), location);
    }
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

// Applies BasicError::map to the error of `result` if it failed.
template <ErrorCodes... Fs, typename T, ErrorCode... Es, std::invocable<Error<Fs...>> F> requires(BasicError<Es...>::template narrows_to<Fs...>)
[[nodiscard]]
auto map_error(BasicResult<T, BasicError<Es...>> &&result, F &&f) {
    return std::move(result).transform_error([&](BasicError<Es...> error) {
        return std::move(error).template map<Fs...>(std::forward<F>(f));
    });
}

// Applies BasicError::map to the error of `result` if it failed.
template <ErrorCodes... Fs, typename T, ErrorCode... Es, ErrorCode E> requires(BasicError<Es...>::template narrows_to<Fs...>)
[[nodiscard]]
auto map_error(BasicResult<T, BasicError<Es...>> &&result, E code, const std::source_location location = std::source_location::current()) {
    return std::move(result).transform_error([&](BasicError<Es...> error) {
        return std::move(error).template map<Fs...>(std::move(code), location);
    });
}

// Applies BasicError::map_code to the error of `result` if it failed.
template <typename T, ErrorCode... Es, typename V, std::invocable<BasicError<Es...>> F> requires(BasicError<Es...>::template compares_with<V>)
[[nodiscard]]
auto map_code(BasicResult<T, BasicError<Es...>> &&result, const V &value, F &&f) {
    return std::move(result).transform_error([&](BasicError<Es...> error) {
        return std::move(error).map_code(value, std::forward<F>(f));
    });
}

// Applies BasicError::map_code to the error of `result` if it failed.
template <typename T, ErrorCode... Es, typename V, ErrorCode E> requires(BasicError<Es...>::template compares_with<V>)
[[nodiscard]]
auto map_code(BasicResult<T, BasicError<Es...>> &&result, const V &value, E code, const std::source_location location = std::source_location::current()) {
    return std::move(result).transform_error([&](BasicError<Es...> error) {
        return std::move(error).map_code(value, std::move(code), location);
    });
}

// The failure of `result`, if it failed, as an error with `code` that keeps it as its source.
template <typename T, ErrorCode... Es, ErrorCode E>
[[nodiscard]]
Result<T, E> wrap_error(BasicResult<T, BasicError<Es...>> &&result, E code, const std::source_location location = std::source_location::current()) {
    return std::move(result).transform_error([&](BasicError<Es...> error) {
        return make_error(std::move(code), std::move(error), location);
    });
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
