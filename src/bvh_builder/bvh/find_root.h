#pragma once

#include <algorithm>
#include <optional>
#include <ranges>
#include <vector>

#include <libassert/assert.hpp>

#include "error/Result.h"
#include "error/try.h"
#include "io/serde.h"
#include "range_utils.h"

#include "bvh/Storage.h"
#include "bvh/Batch.h"
#include "bvh/Reference.h"

namespace bvh {

// Finds the root of `batch` (the one node that no other one references).
// Returns nothing if `batch` is empty or has multiple roots.
[[nodiscard]]
inline std::optional<Reference> find_root(const Batch &batch) {
    if (batch.nodes().empty()) {
        return std::nullopt;
    }

    std::vector<bool> referenced(batch.nodes().size(), false);
    const auto in_batch = [&](const Reference &child) {
        return child.batch == batch.index();
    };
    for (const Reference &child : batch.children() | std::views::filter(in_batch)) {
        DEBUG_ASSERT(child.node < referenced.size());
        referenced[child.node] = true;
    }

    const auto nodes = range(static_cast<NodeIndex>(referenced.size()));
    const auto is_root = [&](const NodeIndex node) {
        return !referenced[node];
    };
    const auto first_root = std::ranges::find_if(nodes, is_root);
    ASSERT(first_root != nodes.end(), "batch contains a cycle");
    const auto second_root = std::ranges::find_if(std::next(first_root), nodes.end(), is_root);
    if (second_root != nodes.end()) {
        // Several trees, so no single root.
        return std::nullopt;
    }
    return Reference{.batch = batch.index(), .node = *first_root};
}

// Finds the root of the tree stored in `nodes` (the root of its last batch).
// Returns nothing if the storage is empty or its last batch has no single root.
[[nodiscard]]
inline alp::Result<std::optional<Reference>, io::serde::Error> find_root(const Storage &nodes) {
    // Builders write a parent only after its children, through one writer at a time, so the root is written last and lands in the last batch.
    if (nodes.size() == 0) {
        return std::nullopt;
    }
    ALP_TRY(const Batch batch, nodes.read(nodes.size() - 1));
    return find_root(batch);
}

}
