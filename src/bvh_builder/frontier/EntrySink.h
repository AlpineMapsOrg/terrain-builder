#pragma once

#include "io/serde.h"
#include "sink/SpanSink.h"

#include "frontier/layout.h"

namespace frontier {

// Takes a stream of entries in spans.
template <typename S>
concept EntrySink = SpanSink<S, Entry, io::serde::Error>;

} // namespace frontier
