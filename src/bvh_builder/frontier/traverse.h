#pragma once

#include <cstdint>
#include <expected>
#include <utility>

#include "Range.h"
#include "io/serde.h"
#include "frontier/layout.h"
#include "frontier/Storage.h"
#include "error/Result.h"
#include "error/try.h"
#include "visitor.h"

namespace frontier {

// Streams the run's batches one at a time. Reports whether it reached the end.
template <typename F>
[[nodiscard]]
alp::Result<bool, io::serde::Error> traverse_batches(const Storage &run, F &&f, const AnyRange<uint32_t> batches = RangeFull{}) {
    visitor::FallibleVisitor<io::serde::Error, F &> visitor(f);

    const Range<uint32_t> range = batches.to_range(run.size());

    for (uint32_t index = range.start; index < range.end; index++) {
        ALP_TRY(const Batch &batch, run.read(index));
        ALP_TRY(const bool keep_going, visitor(batch));
        if (!keep_going) {
            return false;
        }
    }

    return true;
}

// Streams the run's batches one at a time, visiting every entry in order.
// Reports whether it reached the end.
template <typename F>
[[nodiscard]]
alp::Result<bool, io::serde::Error> traverse_entries(const Storage &run, F &&f, const AnyRange<uint32_t> batches = RangeFull{}) {
    visitor::FallibleVisitor<io::serde::Error, F &> visitor(f);

    return traverse_batches(run, [&](const Batch &batch) -> alp::Result<bool, io::serde::Error> {
        for (const Entry &entry : batch.entries) {
            ALP_TRY(const bool keep_going, visitor(entry));
            if (!keep_going) {
                return false;
            }
        }
        return true;
    }, batches);
}

}
