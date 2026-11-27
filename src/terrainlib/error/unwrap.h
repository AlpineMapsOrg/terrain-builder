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

#include <source_location>
#include <stdexcept>
#include <string>
#include <utility>

#include <fmt/format.h>
#include <libassert/assert.hpp>

#include "error/Context.h"
#include "error/Error.h"
#include "error/ErrorCode.h"
#include "error/Result.h"
#include "error/format.h"

namespace alp {

// An unwrapped result that held an error; its message is the error's report.
class UnwrapError : public std::runtime_error {
public:
    template <ErrorCode... Es>
    explicit UnwrapError(const BasicError<Es...> &error) : std::runtime_error(fmt::format("{:?}", error)) {}
};

// The value of `result`; fails an assertion with the error's report if it holds an error.
template <typename T, ErrorCode... Es>
T unwrap(BasicResult<T, BasicError<Es...>> result, const std::source_location location = std::source_location::current()) {
    if (!result.has_value()) {
        PANIC(fmt::format("{:?}", std::move(result).error().propagate(location)));
    }
    return std::move(result).value();
}

// The value of `result`; fails an assertion with the error's report, with `context` as its outermost frame, if it holds an error.
template <typename T, ErrorCode... Es>
T expect(BasicResult<T, BasicError<Es...>> result, Context auto &&context, const std::source_location location = std::source_location::current()) {
    if (!result.has_value()) {
        PANIC(fmt::format("{:?}", std::move(result).error().context(std::forward<decltype(context)>(context), location)));
    }
    return std::move(result).value();
}

// The value of `result`; throws an UnwrapError if it holds an error.
template <typename T, ErrorCode... Es>
T throwing_unwrap(BasicResult<T, BasicError<Es...>> result, const std::source_location location = std::source_location::current()) {
    if (!result.has_value()) {
        throw UnwrapError(std::move(result).error().propagate(location));
    }
    return std::move(result).value();
}

// The value of `result`; throws an UnwrapError with `context` as the error's outermost frame if it holds an error.
template <typename T, ErrorCode... Es>
T throwing_expect(BasicResult<T, BasicError<Es...>> result, Context auto &&context, const std::source_location location = std::source_location::current()) {
    if (!result.has_value()) {
        throw UnwrapError(std::move(result).error().context(std::forward<decltype(context)>(context), location));
    }
    return std::move(result).value();
}

// `result` without the codes Fs among its errors; fails an assertion with the error's report if it holds one of them.
template <ErrorCodes... Fs, typename T, ErrorCode... Es>
auto unwrap_partial(BasicResult<T, BasicError<Es...>> result, const std::source_location location = std::source_location::current()) {
    return std::move(result).transform_error([&](BasicError<Es...> error) {
        auto split_result = std::move(error).template split<Fs...>();
        if (split_result.has_value()) {
            PANIC(fmt::format("{:?}", std::move(split_result).value().propagate(location)));
        }
        return std::move(split_result).error();
    });
}

// `result` without the codes Fs among its errors; fails an assertion with the error's report, with `context` as its outermost frame, if it holds one of them.
template <ErrorCodes... Fs, typename T, ErrorCode... Es>
auto expect_partial(BasicResult<T, BasicError<Es...>> result, Context auto &&context, const std::source_location location = std::source_location::current()) {
    return std::move(result).transform_error([&](BasicError<Es...> error) {
        auto split_result = std::move(error).template split<Fs...>();
        if (split_result.has_value()) {
            PANIC(fmt::format("{:?}", std::move(split_result).value().context(std::forward<decltype(context)>(context), location)));
        }
        return std::move(split_result).error();
    });
}

// The value of `result`, which cannot hold a failure.
template <typename T>
T unwrap_infallible(BasicResult<T, Error<Infallible>> result) {
    return std::move(result).value();
}

// `result` itself, which may hold a failure.
template <typename T, typename E>
BasicResult<T, E> unwrap_infallible(BasicResult<T, E> result) {
    return result;
}

} // namespace alp
