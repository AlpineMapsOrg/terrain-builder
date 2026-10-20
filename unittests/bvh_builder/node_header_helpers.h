#pragma once

#include <array>

#include "Range.h"

#include "bvh/NodeHeader.h"
#include "bvh/NodeInput.h"
#include "dag/ClusterInfo.h"

// A leaf input with `header`, holding a cluster that nothing reads.
inline auto make_leaf_input(const bvh::NodeHeader &header = {}) {
    return bvh::LeafInput{header, std::array{dag::ClusterInfo{}}};
}

inline dag::ClusterInfo make_empty_cluster(const dag::Id &id) {
    dag::ClusterInfo cluster{};
    cluster.id = id;
    return cluster;
}

// A node header with the error interval `errors`.
inline bvh::NodeHeader make_node_header(const RangeInclusive<double> &errors, const bvh::Bounds &lod_bounds, const bvh::Bounds &tight_bounds) {
    return bvh::NodeHeader{
        .min_child_error = errors.start,
        .max_parent_error = errors.end,
        .lod_bounds = lod_bounds,
        .tight_bounds = tight_bounds,
    };
}

// A node header with the error interval `errors`, whose LOD and tight bounds are both `bounds`.
inline bvh::NodeHeader make_node_header(const RangeInclusive<double> &errors, const bvh::Bounds &bounds) {
    return make_node_header(errors, bounds, bounds);
}
