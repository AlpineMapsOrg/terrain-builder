#pragma once

#include <compare>
#include <limits>

#include <fmt/format.h>
#include <radix/geometry.h>

#include "fmt_impls.h"
#include "geometry/compare.h"

namespace bvh {

using Bounds = radix::geometry::Aabb3d;

// The parent error of a leaf whose clusters nothing replaces.
constexpr double ROOT_PARENT_ERROR = std::numeric_limits<double>::infinity();

struct NodeHeader {
    double min_child_error; // the smallest error of the groups that produced the clusters below
    double max_parent_error; // the largest error of the groups listing the clusters below, ROOT_PARENT_ERROR if no group lists some of them
    Bounds lod_bounds; // the box the errors are projected from
    Bounds tight_bounds; // a tight box around the subtree's geometry, for culling

    bool operator==(const NodeHeader &) const = default;

    // Orders headers by their error interval, then by their bounds.
    std::strong_ordering operator<=>(const NodeHeader &other) const {
        return geometry::tie_strong(this->min_child_error, this->max_parent_error, this->lod_bounds, this->tight_bounds) <=>
               geometry::tie_strong(other.min_child_error, other.max_parent_error, other.lod_bounds, other.tight_bounds);
    }
};

}

template <>
struct fmt::formatter<bvh::NodeHeader> {
    constexpr auto parse(fmt::format_parse_context &ctx) {
        return ctx.begin();
    }
    auto format(const bvh::NodeHeader &header, fmt::format_context &ctx) const {
        return fmt::format_to(
            ctx.out(),
            "NodeHeader(min_child_error={}, max_parent_error={}, lod_bounds={}, tight_bounds={})",
            header.min_child_error,
            header.max_parent_error,
            header.lod_bounds,
            header.tight_bounds);
    }
};
