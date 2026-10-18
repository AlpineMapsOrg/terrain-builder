#pragma once

#include "io/Error.h"
#include "sink/SpanSink.h"

#include "frontier/layout.h"

namespace frontier {

// Takes a stream of entries in spans.
template <typename S>
concept EntrySink = SpanSink<S, Entry, io::Error>;

} // namespace frontier
