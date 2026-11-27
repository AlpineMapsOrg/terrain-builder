#pragma once

#include <concepts>
#include <cstdint>
#include <expected>
#include <type_traits>

#include "enumerate.h"
#include "io/serde.h"
#include "range_utils.h"
#include "bvh/Batch.h"
#include "bvh/Reference.h"
#include "bvh/Storage.h"
#include "visitor.h"
#include "error/Result.h"
#include "error/try.h"

namespace bvh {

// Visits every stored node with the reference naming it. False if the visitor stopped the walk before its end.
template <typename F>
[[nodiscard]]
alp::Result<bool, io::serde::Error> flat_traverse(const Storage &nodes, F &&v) {
    visitor::FallibleVisitor<io::serde::Error, F &> visitor(v);

    for (const BatchIndex batch_index : range(nodes.size())) {
        ALP_TRY(const Batch batch, nodes.read(batch_index));

        for (const auto &[node_index, node] : enumerate<NodeIndex>(batch.nodes())) {
            const Reference ref{.batch = batch_index, .node = node_index};
            ALP_TRY(const bool keep_going, visitor(ref, node));
            if (!keep_going) {
                return false;
            }
        }
    }

    // We walked to the end
    return true;
}

}
