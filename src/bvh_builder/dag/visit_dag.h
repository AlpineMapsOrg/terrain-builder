#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>

#include "enumerate.h"
#include "error/Result.h"
#include "error/try.h"
#include "io/Error.h"
#include "octree/Id.h"

#include "dag_id.h"
#include "metadata.h"
#include "../../dag_builder/storage.h"
#include "dag/Cluster.h"
#include "dag/Group.h"
#include "dag/NodeMetadataCache.h"
#include "dag/read.h"
#include "visitor.h"

namespace dag {

namespace detail {

// Lists the batches of `index`.
inline std::vector<BatchId> collect_batches(const octree::IndexMap &index) {
    std::vector<BatchId> result;
    result.reserve(index.size());
    for (const auto &[batch, status] : index) {
        if (index.has_data(batch)) {
            result.push_back(batch);
        }
    }
    return result;
}

// Lists the batches on the lowest level of `index`.
inline std::vector<BatchId> collect_root_batches(const octree::IndexMap &index) {
    std::vector<BatchId> result;
    octree::Id::Level min_level = octree::Id::max_level();
    for (const auto &[batch, status] : index) {
        if (!index.has_data(batch)) {
            // Virtual nodes have no batch.
            continue;
        }

        if (batch.level() > min_level) {
            // Not on the lowest level so far.
            continue;
        }

        if (batch.level() < min_level) {
            // A new lowest level drops the batches found so far.
            min_level = batch.level();
            result.clear();
        }

        // On the lowest level so far.
        result.push_back(batch);
    }
    return result;
}

} // namespace detail

// Visits every group of the dag.
// False if the visitor stopped the walk before its end.
template <typename F>
[[nodiscard]]
alp::Result<bool, io::Error> visit_groups(F &&v, NodeMetadataCache &cache) {
    visitor::FallibleVisitor<io::Error, F &> visitor(v);

    std::vector<BatchId> batches = detail::collect_batches(cache.storage().index());
    // Sorting by global index keeps groups that are close in space close in the walk.
    std::ranges::sort(batches, {}, &octree::Id::global_index);
    for (const BatchId &batch : batches) {
        ALP_TRY(const std::shared_ptr<const NodeMetadata> batch_metadata, cache.read_metadata(batch));
        for (const auto &[group_index, group_metadata] : enumerate<uint32_t>(batch_metadata->groups)) {
            const Group group(GroupId{.batch = batch, .group_index = group_index}, batch_metadata);
            ALP_TRY(const bool keep_going, visitor(group));
            if (!keep_going) {
                return false;
            }
        }
    }
    return true;
}

// Visits every cluster no group lists, that is every cluster on the lowest level.
// False if the visitor stopped the walk before its end.
template <typename F>
[[nodiscard]]
alp::Result<bool, io::Error> visit_root_clusters(F &&v, NodeMetadataCache &cache) {
    visitor::FallibleVisitor<io::Error, F &> visitor(v);

    std::vector<BatchId> batches = detail::collect_root_batches(cache.storage().index());
    // Sorting by global index keeps the order of the walk the same in every run.
    std::ranges::sort(batches, {}, &octree::Id::global_index);
    for (const BatchId &batch : batches) {
        ALP_TRY(const std::shared_ptr<const NodeMetadata> batch_metadata, cache.read_metadata(batch));
        for (const auto &[cluster_index, cluster_metadata] : enumerate<uint32_t>(batch_metadata->clusters)) {
            ALP_TRY(const Cluster cluster, read_cluster(ClusterId{.batch = batch, .cluster_index = cluster_index}, cache));
            ALP_TRY(const bool keep_going, visitor(cluster));
            if (!keep_going) {
                return false;
            }
        }
    }
    return true;
}

} // namespace dag
