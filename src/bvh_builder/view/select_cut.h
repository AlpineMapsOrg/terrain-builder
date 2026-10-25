#pragma once

#include <algorithm>
#include <cstdint>
#include <span>
#include <vector>

#include "error/Result.h"
#include "error/try.h"
#include "io/Error.h"

#include "bvh/NodeCache.h"
#include "bvh/NodeHeader.h"
#include "bvh/NodeView.h"
#include "bvh/Reference.h"
#include "bvh/traverse_tree.h"
#include "dag/Cluster.h"
#include "dag/NodeMetadataCache.h"
#include "dag/SourceId.h"
#include "dag/read.h"
#include "view/Camera.h"
#include "view/CutRule.h"
#include "view/CutView.h"

namespace view {

// The groups selected for a camera, with the work it took to find them.
struct GroupSelection {
    // Every group the cut can cross.
    std::vector<dag::SourceId> groups;
    // How many inner nodes were checked.
    uint64_t inner_nodes_tested;
    // How many leaves were checked.
    uint64_t leaves_tested;

    bool operator==(const GroupSelection &) const = default;
};

// Selects the groups the cut for `camera` under `rule` can cross, from the tree at `root`.
// Skips every subtree that cannot hold any of them.
// Fails if a batch cannot be read.
[[nodiscard]]
inline alp::Result<GroupSelection, io::Error> select_groups(bvh::NodeCache &nodes, const bvh::Reference &root, const Camera &camera, const CutRule &rule) {
    const CutView view(camera, rule);

    GroupSelection selection{};
    const auto visit = [&](const bvh::Reference &, const bvh::NodeView &node) {
        const bvh::NodeHeader &header = node.header();
        return node.visit(
            [&](const dag::SourceId &source) {
                selection.leaves_tested++;
                if (view.could_cross_leaf(header)) {
                    selection.groups.push_back(source);
                }
                return bvh::Control::Continue;
            },
            [&](const std::span<const bvh::Reference>) {
                selection.inner_nodes_tested++;
                return view.could_contain_cut(header) ? bvh::Control::Continue : bvh::Control::Prune;
            });
    };
    ALP_TRY(bvh::traverse_tree(nodes, root, visit));
    return selection;
}

// Selects the clusters of `groups` on the cut for `camera` under `rule`, sorted.
// Fails if the metadata of a batch cannot be read.
[[nodiscard]]
inline alp::Result<std::vector<dag::ClusterId>, io::Error> select_clusters(std::span<const dag::SourceId> groups, dag::NodeMetadataCache &cache, const Camera &camera, const CutRule &rule) {
    const CutView view(camera, rule);

    std::vector<dag::ClusterId> result;
    for (const dag::SourceId &group : groups) {
        ALP_TRY(const std::vector<dag::Cluster> clusters, dag::read_source_clusters(group, cache));
        // A cluster shows while the group that produced it is small enough and the group listing it is not.
        for (const dag::Cluster &cluster : clusters) {
            if (view.is_small_enough_to_draw(cluster.error(), cluster.lod_bounds()) && view.is_in_view(cluster.tight_bounds())) {
                result.push_back(cluster.id());
            }
        }
    }
    std::ranges::sort(result);
    return result;
}

}
