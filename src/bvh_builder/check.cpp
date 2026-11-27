#include <cstddef>
#include <cstdint>
#include <optional>
#include <ranges>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <fmt/std.h>

#include "cli.h"
#include "commands.h"
#include "error/Context.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "geometry/bounds.h"
#include "log.h"
#include "bvh/Batch.h"
#include "bvh/NodeCache.h"
#include "bvh/NodeHeader.h"
#include "bvh/NodeView.h"
#include "bvh/Reference.h"
#include "bvh/Storage.h"
#include "bvh/find_root.h"
#include "bvh/flat_traverse.h"

namespace {

// Marks the stored nodes a walk reached, one list per batch, empty until the walk first reads that batch.
using ReachedNodes = std::vector<std::vector<bool>>;

// A node the walk still has to check, with the header of the parent listing it, none for the root.
struct PendingNode {
    bvh::Reference reference;
    std::optional<bvh::NodeHeader> parent_header_opt;
};

// Whether the error interval of `parent` spans that of `child` and its boxes contain the child's.
bool covers(const bvh::NodeHeader &parent, const bvh::NodeHeader &child) {
    return parent.min_child_error <= child.min_child_error && parent.max_parent_error >= child.max_parent_error &&
           geometry::union_bounds(parent.lod_bounds, child.lod_bounds) == parent.lod_bounds &&
           geometry::union_bounds(parent.tight_bounds, child.tight_bounds) == parent.tight_bounds;
}

// Walks the tree from `root`, printing every reference to a node that is not stored, every node reached more than once, every node its parent does not cover and every leaf whose errors are not monotone.
// Marks the nodes it reached in `reached`. Returns how many violations it printed.
alp::Result<uint64_t, io::Error> check_tree(bvh::NodeCache &cache, const bvh::Reference &root, const size_t batch_count, ReachedNodes &reached) {
    uint64_t violation_count = 0;
    std::vector<PendingNode> stack{PendingNode{.reference = root, .parent_header_opt = std::nullopt}};
    while (!stack.empty()) {
        const PendingNode pending = std::move(stack.back());
        stack.pop_back();
        const bvh::Reference &reference = pending.reference;

        if (reference.batch >= batch_count) {
            fmt::print("node {}/{}: its batch is not stored\n", reference.batch, reference.node);
            violation_count++;
            continue;
        }
        ALP_TRY(const std::shared_ptr<const bvh::Batch> shared_batch, cache.read_batch(reference.batch));
        const bvh::Batch &batch = *shared_batch;
        std::vector<bool> &batch_reached = reached[reference.batch];
        if (batch_reached.empty()) {
            batch_reached.resize(batch.nodes().size());
        }
        if (reference.node >= batch_reached.size()) {
            fmt::print("node {}/{}: it is not stored\n", reference.batch, reference.node);
            violation_count++;
            continue;
        }
        if (batch_reached[reference.node]) {
            fmt::print("node {}/{}: it is reached more than once\n", reference.batch, reference.node);
            violation_count++;
            continue;
        }
        batch_reached[reference.node] = true;

        const bvh::NodeView view = batch.resolve(batch.node(reference.node));
        if (pending.parent_header_opt.has_value() && !covers(*pending.parent_header_opt, view.header())) {
            fmt::print("node {}/{}: its parent's {} does not cover its {}\n", reference.batch, reference.node, *pending.parent_header_opt, view.header());
            violation_count++;
        }
        // Negated so that a NaN error counts as a violation too.
        if (view.is_leaf() && !(view.header().min_child_error <= view.header().max_parent_error)) {
            fmt::print("node {}/{}: its errors are not monotone in {}\n", reference.batch, reference.node, view.header());
            violation_count++;
        }

        // Pushed in reverse so that they come off the stack in stored order.
        if (view.is_inner()) {
            for (const bvh::Reference &child : view.children() | std::views::reverse) {
                stack.push_back(PendingNode{.reference = child, .parent_header_opt = view.header()});
            }
        }
    }
    return violation_count;
}

// Prints every stored node that `reached` does not mark. Returns how many it printed.
alp::Result<uint64_t, io::Error> check_reached(const bvh::Storage &nodes, const ReachedNodes &reached) {
    uint64_t violation_count = 0;
    ALP_TRY(bvh::flat_traverse(nodes, [&](const bvh::Reference &reference, const bvh::Node &) {
        const std::vector<bool> &batch_reached = reached[reference.batch];
        // A batch the walk never read has no marks.
        if (batch_reached.empty() || !batch_reached[reference.node]) {
            fmt::print("node {}/{}: it is stored but not in the tree\n", reference.batch, reference.node);
            violation_count++;
        }
    }));
    return violation_count;
}

}

alp::ErasedResult<void> check_bvh(const cli::CheckArgs &args) {
    ALP_TRY(const bvh::Storage nodes, bvh::Storage::open(args.input_path));
    ALP_TRY(const std::optional<bvh::Reference> root_opt, bvh::find_root(nodes));
    const auto missing_root_context = alp::make_context("{} holds a forest or is empty", args.input_path);
    ALP_TRY(const bvh::Reference root, alp::to_result(root_opt, missing_root_context));

    bvh::NodeCache cache(nodes, args.max_resident_bytes);
    ReachedNodes reached(nodes.size());
    ALP_TRY(const uint64_t tree_violation_count, check_tree(cache, root, nodes.size(), reached));
    ALP_TRY(const uint64_t unreached_count, check_reached(nodes, reached));
    LOG_INFO("The walk read {} batches from disk for {} stored batches", cache.count_misses(), nodes.size());

    const uint64_t violation_count = tree_violation_count + unreached_count;
    if (violation_count != 0) {
        return alp::make_error(fmt::format("{} has {} violations", args.input_path.string(), violation_count));
    }
    LOG_INFO("{} is one tree whose headers cover their children", args.input_path.string());
    return {};
}
