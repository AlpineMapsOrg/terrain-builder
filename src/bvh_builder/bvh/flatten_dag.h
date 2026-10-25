#pragma once

#include <cstddef>
#include <expected>
#include <span>

#include <libassert/assert.hpp>
#include <radix/geometry.h>

#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/Error.h"
#include "range_utils.h"
#include "variant_utils.h"

#include "../../dag_builder/storage.h"

#include "bvh/NodeHeader.h"
#include "bvh/NodeInput.h"
#include "bvh/Storage.h"
#include "bvh/BatchWriter.h"
#include "dag/traverse_dag.h"

namespace bvh {

struct FlattenOptions {
    size_t max_batch_bytes = DEFAULT_MAX_BATCH_BYTES; // the most bytes a written batch holds
    size_t max_resident_bytes = DEFAULT_MAX_RESIDENT_BYTES; // the most bytes of dag node metadata kept cached
};

namespace detail {

// Makes the leaf holding the child clusters of `group`.
inline LeafInput<std::span<const dag::ClusterInfo>> make_leaf(const dag::GroupView &group) {
    DEBUG_ASSERT(!group.child_clusters.empty());

    radix::geometry::Aabb3d tight_bounds;
    for (const dag::ClusterInfo &cluster : group.child_clusters) {
        tight_bounds.expand_by(cluster.tight_bounds);
    }

    const double max_parent_error = match(
        group.id,
        [&](const dag::GroupId &) { return group.error; },
        [](const dag::ClusterId &) { return ROOT_PARENT_ERROR; });
    const NodeHeader header{
        .min_child_error = min(group.child_clusters, &dag::ClusterInfo::error),
        .max_parent_error = max_parent_error,
        .lod_bounds = group.bounds,
        .tight_bounds = tight_bounds,
    };
    return LeafInput{header, group.child_clusters};
}

} // namespace detail

// Writes the leaves of a bvh over `dag` into `output`.
[[nodiscard]]
inline alp::Result<void, io::Error> flatten_dag(const octree::IndexedDagMetaStorage &dag, Storage &output, const FlattenOptions &options = {}) {
    return alp::run_with_context(
        alp::make_context("flattening a dag from {} to {}", dag.base_path(), output.directory()),
        [&] -> alp::Result<void, io::Error> {
            BatchWriter writer(output, options.max_batch_bytes);
            ALP_TRY(dag::traverse_groups(dag, options.max_resident_bytes, [&](const dag::GroupView &group) -> alp::Result<void, io::Error> {
                // The clusters a group lists form one leaf.
                if (!group.child_clusters.empty()) {
                    ALP_TRY(writer.write_node(detail::make_leaf(group)));
                }
                return {};
            }));
            return writer.flush();
        });
}

}
