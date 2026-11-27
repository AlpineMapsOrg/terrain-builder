#pragma once

#include <glm/glm.hpp>
#include <radix/geometry.h>

#include "geometry/bounds.h"

#include "bvh/NodeHeader.h"

namespace build {

const double DEFAULT_ERROR_WEIGHT = 1.0;

// Makes the box a node's cost is measured on: `bounds` with the error interval as a fourth axis, scaled by `error_weight`.
[[nodiscard]]
inline radix::geometry::Aabb<4, double> make_cost_bounds(const bvh::Bounds &bounds, const double min_child_error, const double max_parent_error, const double error_weight) {
    // A root's unbounded parent error would swallow every other axis.
    const double parent_error = max_parent_error == bvh::ROOT_PARENT_ERROR ? min_child_error : max_parent_error;
    return {
        glm::dvec4(bounds.min, error_weight * min_child_error),
        glm::dvec4(bounds.max, error_weight * parent_error),
    };
}

// Makes the box a node's cost is measured on, from its tight bounds and error interval.
[[nodiscard]]
inline radix::geometry::Aabb<4, double> make_cost_bounds(const bvh::NodeHeader &header, const double error_weight = DEFAULT_ERROR_WEIGHT) {
    return make_cost_bounds(header.tight_bounds, header.min_child_error, header.max_parent_error, error_weight);
}

// Computes the cost of one node covering `bounds`.
[[nodiscard]]
inline double compute_node_cost(const radix::geometry::Aabb<4, double> &bounds) {
    return geometry::compute_surface_area(bounds);
}

// Computes the cost of keeping `header` as a node of its tree, with its error interval weighed in by `error_weight`.
[[nodiscard]]
inline double compute_node_cost(const bvh::NodeHeader &header, const double error_weight = DEFAULT_ERROR_WEIGHT) {
    return compute_node_cost(make_cost_bounds(header, error_weight));
}

// Computes the cost of putting both extended nodes under one parent.
[[nodiscard]]
inline double compute_merge_cost(
    const radix::geometry::Aabb<4, double> &left, const radix::geometry::Aabb<4, double> &right) {
    radix::geometry::Aabb<4, double> merged = left;
    merged.expand_by(right);
    return compute_node_cost(merged);
}

// Computes the cost of putting both nodes under one parent, with error as a fourth axis.
[[nodiscard]]
inline double compute_merge_cost(const bvh::NodeHeader &left, const bvh::NodeHeader &right, const double error_weight = DEFAULT_ERROR_WEIGHT) {
    return compute_merge_cost(make_cost_bounds(left, error_weight), make_cost_bounds(right, error_weight));
}

} // namespace build
