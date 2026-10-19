#pragma once

#include <expected>

#include "error/Error.h"
#include "error/ErrorCode.h"

namespace alp {

// The underlying result type; a value, or an error of type E.
template <typename T, typename E>
using BasicResult = std::expected<T, E>;

// A value, or an error with codes of type Es; with no Es, an erased error.
template <typename T, typename... Es>
using Result = BasicResult<T, Error<Es...>>;

// The underlying unexpected type; an error of type E, to return from a function that returns a BasicResult.
template <typename E>
using BasicUnexpected = std::unexpected<E>;

// An error with codes of type Es, to return from a function that returns a Result.
template <typename... Es>
using Unexpected = BasicUnexpected<Error<Es...>>;

} // namespace alp
