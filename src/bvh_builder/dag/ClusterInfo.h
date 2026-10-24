#pragma once

#include <fmt/format.h>
#include <radix/geometry.h>

#include "fmt_impls.h"

#include "dag_id.h"

namespace dag {

// A cluster of the dag.
struct ClusterInfo {
    Id id;
    double error; // the error of its group, 0 for a leaf cluster
    radix::geometry::Aabb3d lod_bounds; // the box its error is measured from: its group's, or its own for a leaf cluster
    radix::geometry::Aabb3d tight_bounds; // a tight box around it, for culling

    bool operator==(const ClusterInfo &) const = default;
};

} // namespace dag

template <>
struct fmt::formatter<dag::ClusterInfo> {
    constexpr auto parse(fmt::format_parse_context &ctx) {
        return ctx.begin();
    }
    auto format(const dag::ClusterInfo &cluster, fmt::format_context &ctx) const {
        return fmt::format_to(ctx.out(), "ClusterInfo(id={}, error={}, lod_bounds={}, tight_bounds={})", cluster.id, cluster.error, cluster.lod_bounds, cluster.tight_bounds);
    }
};
