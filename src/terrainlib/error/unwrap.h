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
class Panic : public std::runtime_error {
public:
    template <ErrorCode... Es>
    explicit Panic(const BasicError<Es...> &error) : std::runtime_error(fmt::format("{:?}", error)) {}
};

// The value of `result`; throws a Panic if it holds an error.
template <typename T, ErrorCode... Es>
T throwing_unwrap(BasicResult<T, BasicError<Es...>> result, const std::source_location location = std::source_location::current()) {
    if (!result.has_value()) {
        throw Panic(std::move(result).error().propagate(location));
    }
    return std::move(result).value();
}

// The value of `result`; throws a Panic with `context` as the error's outermost frame if it holds an error.
template <typename T, ErrorCode... Es>
T throwing_expect(BasicResult<T, BasicError<Es...>> result, Context auto &&context, const std::source_location location = std::source_location::current()) {
    if (!result.has_value()) {
        throw Panic(std::move(result).error().context(std::forward<decltype(context)>(context), location));
    }
    return std::move(result).value();
}

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

} // namespace alp
