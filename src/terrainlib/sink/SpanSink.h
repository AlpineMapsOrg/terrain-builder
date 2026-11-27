#pragma once

#include <span>

#include "sink/Sink.h"

// A sink that takes its values in spans.
template <typename S, typename T, typename... Es>
concept SpanSink = Sink<S, std::span<const T>, Es...>;
