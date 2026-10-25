#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "cli.h"
#include "commands.h"
#include "dag_id.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "bvh/Node.h"
#include "bvh/NodeCache.h"
#include "bvh/Reference.h"
#include "bvh/Storage.h"
#include "bvh/find_root.h"
#include "bvh/flat_traverse.h"
#include "json/glm.h"
#include "json/octree.h"
#include "json/serialize.h"
#include "view/Camera.h"
#include "view/CameraOptions.h"
#include "view/CameraSet.h"
#include "view/CutRule.h"
#include "view/select_cut.h"

namespace {

// The clusters a camera selected and the work selecting them took.
struct CutStats {
    uint64_t cluster_count;
    uint64_t inner_nodes_tested;
    uint64_t leaves_tested;
};

// The cuts a camera set selects from a bvh.
struct Evaluation {
    std::string camera_set_name;
    view::CutRule rule;
    uint64_t leaf_count;
    // In the order of the set's cameras.
    std::vector<CutStats> stats = {};
    // In the order of the set's cameras.
    std::optional<std::vector<std::vector<dag::ClusterId>>> cuts = std::nullopt;
};

}

alp::Result<void> evaluate_bvh(const cli::EvaluateArgs &args) {
    ALP_TRY(const bvh::Storage nodes, bvh::Storage::open(args.input_path));
    ALP_TRY(const std::optional<bvh::Reference> root_opt, bvh::find_root(nodes));
    if (!root_opt.has_value()) {
        return alp::make_unexpected(fmt::format("{} holds a forest or is empty", args.input_path.string()));
    }

    ALP_TRY(const view::CameraSet camera_set, json::read_from_path<view::CameraSet>(args.cameras_path));
    std::vector<view::Camera> cameras;
    cameras.reserve(camera_set.cameras.size());
    for (const view::CameraOptions &options : camera_set.cameras) {
        ALP_TRY(const view::Camera camera, view::Camera::create(options));
        cameras.push_back(camera);
    }

    uint64_t leaf_count = 0;
    ALP_TRY(bvh::flat_traverse(nodes, [&](const bvh::Reference &, const bvh::Node &node) {
        if (node.is_leaf()) {
            leaf_count++;
        }
    }));

    const view::CutRule rule{.threshold = args.threshold, .cull = !args.no_cull};
    bvh::NodeCache cache(nodes, args.max_resident_bytes);
    Evaluation evaluation{.camera_set_name = camera_set.name, .rule = rule, .leaf_count = leaf_count};
    if (args.write_cut) {
        evaluation.cuts.emplace();
    }
    evaluation.stats.reserve(cameras.size());
    for (const view::Camera &camera : cameras) {
        ALP_TRY(view::Selection selection, view::select_cut(cache, *root_opt, camera, rule));
        evaluation.stats.push_back(CutStats{
            .cluster_count = selection.clusters.size(),
            .inner_nodes_tested = selection.inner_nodes_tested,
            .leaves_tested = selection.leaves_tested,
        });
        if (evaluation.cuts.has_value()) {
            evaluation.cuts->push_back(std::move(selection.clusters));
        }
    }

    if (args.output_path.has_value()) {
        return json::write_to_path(evaluation, *args.output_path);
    }
    ALP_TRY(const std::string text, json::write_to_text(evaluation));
    fmt::print("{}\n", text);
    return {};
}
