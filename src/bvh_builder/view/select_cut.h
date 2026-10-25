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
#include "dag/ClusterInfo.h"
#include "view/Camera.h"
#include "view/CutRule.h"
#include "view/Frustum.h"

namespace view {

// The cut selected for a camera, with the work it took to find.
struct Selection {
    // Every cluster selected, sorted.
    std::vector<dag::ClusterId> clusters;
    // How many inner nodes were checked.
    uint64_t inner_nodes_tested;
    // How many leaves were checked.
    uint64_t leaves_tested;

    bool operator==(const Selection &) const = default;
};

namespace detail {

// The camera, the rule and what is derived from them, prepared once per view.
struct CutView {
    Camera camera;
    Frustum frustum;
    CutRule rule;
};

// Prepares the view of `camera` under `rule`.
[[nodiscard]]
inline CutView make_cut_view(const Camera &camera, const CutRule &rule) {
    return CutView{.camera = camera, .frustum = Frustum::from_camera(camera), .rule = rule};
}

// Checks whether `bounds` are in view, as they always are if the rule does not cull.
[[nodiscard]]
inline bool is_in_view(const bvh::Bounds &bounds, const CutView &view) {
    return !view.rule.cull || intersects(bounds, view.frustum);
}

// Checks whether the subtree below `header` could contain any of the cut.
[[nodiscard]]
inline bool could_contain_cut(const bvh::NodeHeader &header, const CutView &view) {
    return is_in_view(header.tight_bounds, view) && !is_small_enough_to_draw(header.max_parent_error, header.lod_bounds, view.camera, view.rule);
}

// Adds the leaf's clusters that are on the cut to `selected`.
inline void select_clusters(const bvh::NodeHeader &header, const std::span<const dag::ClusterInfo> clusters, const CutView &view, std::vector<dag::ClusterId> &selected) {
    if (!is_in_view(header.tight_bounds, view) || !should_visit_leaf(header, view.camera, view.rule)) {
        return;
    }
    // A cluster shows while the group that produced it is small enough and the group listing it is not.
    for (const dag::ClusterInfo &cluster : clusters) {
        if (is_small_enough_to_draw(cluster.error, cluster.lod_bounds, view.camera, view.rule) && is_in_view(cluster.tight_bounds, view)) {
            selected.push_back(cluster.id);
        }
    }
}

}

// Selects the clusters for `camera` under `rule` from the tree at `root`.
// Skips every subtree that cannot hold any of them.
// Fails if a batch cannot be read.
[[nodiscard]]
inline alp::Result<Selection, io::Error> select_cut(bvh::NodeCache &nodes, const bvh::Reference &root, const Camera &camera, const CutRule &rule) {
    const detail::CutView view = detail::make_cut_view(camera, rule);

    Selection selection{};
    const auto visit = [&](const bvh::Reference &, const bvh::NodeView &node) {
        const bvh::NodeHeader &header = node.header();
        return node.visit(
            [&](const std::span<const dag::ClusterInfo> clusters) {
                selection.leaves_tested++;
                detail::select_clusters(header, clusters, view, selection.clusters);
                return bvh::Control::Continue;
            },
            [&](const std::span<const bvh::Reference>) {
                selection.inner_nodes_tested++;
                return detail::could_contain_cut(header, view) ? bvh::Control::Continue : bvh::Control::Prune;
            });
    };
    ALP_TRY(bvh::traverse_tree(nodes, root, visit));

    std::ranges::sort(selection.clusters);
    return selection;
}

}
