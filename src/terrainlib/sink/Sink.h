#pragma once

#include <concepts>
#include <type_traits>
#include <utility>

#include "error/Error.h"
#include "error/Result.h"

// Takes a stream of values, failing with at most the codes `Es`; a sink that cannot fail returns results with alp::Infallible. `finish` passes on what is buffered and ends the stream.
template <typename S, typename T, typename... Es>
concept Sink = requires(S &sink, T value) {
    { sink.push(std::move(value)) } -> std::convertible_to<alp::Result<void, Es...>>;
    { std::move(sink).finish() } -> std::convertible_to<alp::Result<void, Es...>>;
};

// What a sink of type S fails with when it takes values of type T.
template <typename S, typename T>
using SinkError = alp::Error<
    typename decltype(std::declval<S &>().push(std::declval<T>()))::error_type,
    typename decltype(std::declval<std::remove_reference_t<S> &&>().finish())::error_type>;
