#pragma once

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <ranges>
#include <span>
#include <utility>

#include <libassert/assert.hpp>

#include "geometry/bounds.h"

#include "bvh/NodeHeader.h"

namespace bvh {

// Computes the header of a node from those of its children.
// There must be at least one.
template <std::ranges::input_range R>
NodeHeader merge_headers(R &&headers) {
    DEBUG_ASSERT(!std::ranges::empty(headers));
    const auto merge =[](const NodeHeader &a, const NodeHeader &b) {
        return NodeHeader{
            .min_child_error = std::min(a.min_child_error, b.min_child_error),
            .max_parent_error = std::max(a.max_parent_error, b.max_parent_error),
            .lod_bounds = geometry::union_bounds(a.lod_bounds, b.lod_bounds),
            .tight_bounds = geometry::union_bounds(a.tight_bounds, b.tight_bounds),
        };
    };
    return std::ranges::fold_left_first(headers, merge).value();
}

} // namespace bvh
