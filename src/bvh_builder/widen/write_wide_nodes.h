#pragma once

#include <array>
#include <cstdint>
#include <expected>
#include <span>
#include <vector>

#include <fmt/format.h>
#include <libassert/assert.hpp>

#include "array_utils.h"
#include "io/Error.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"

#include "bvh/BatchWriter.h"
#include "bvh/NodeCache.h"
#include "bvh/Storage.h"
#include "bvh/Batch.h"
#include "bvh/Node.h"
#include "bvh/NodeInput.h"
#include "bvh/Reference.h"
#include "dag/ClusterInfo.h"
#include "widen/options.h"
#include "widen/tabulate_splits.h"

namespace widen {

namespace detail {

// What a node does with the slots it was offered: how many go to each of its children, and whether it becomes a node of the widened tree.
struct Decision {
    uint8_t left_slots;
    uint8_t right_slots;
    bool kept;
};

// What the node whose splits are `row` does when it is offered `slots`.
[[nodiscard]]
inline Decision compute_decision(const std::span<const Split> row, const uint8_t slots, const Options &options) {
    // Past the end of the row every leaf below can have a slot of its own, so nothing under here is kept.
    // Both children can be offered everything, since in that regime a subtree only ever uses one slot per leaf.
    if (slots > row.size()) {
        const uint8_t max_slots = static_cast<uint8_t>(options.arity - 1);
        return Decision{.left_slots = max_slots, .right_slots = max_slots, .kept = false};
    }

    // A kept node fills one slot of its parent and hands a fresh arity to its own children.
    const Split split = row[slots - 1];
    const uint8_t shared_slots = split.is_kept() ? static_cast<uint8_t>(options.arity) : slots;
    return Decision{
        .left_slots = split.left_slots(),
        .right_slots = split.right_slots(shared_slots),
        .kept = split.is_kept(),
    };
}

// Writes the wide nodes standing for the subtree at `reference` when it is offered `slots`, appending one reference per child it contributes to `children`.
[[nodiscard]]
inline alp::Result<void, io::Error> write_subtree(
    bvh::NodeCache &input, const bvh::Reference &reference, const uint8_t slots,
    SplitReader &rows, bvh::BatchWriter &writer, const Options &options,
    std::vector<bvh::Reference> &children) {
    DEBUG_ASSERT(slots >= 1);

    ALP_TRY(const std::shared_ptr<const bvh::Batch> shared_batch, input.read_batch(reference.batch));
    const bvh::Batch &batch = *shared_batch;
    const bvh::Node node = batch.node(reference.node);

    return batch.visit(
        node,
        // A leaf is carried over as it is, filling the one slot it was offered.
        [&](const std::span<const dag::ClusterInfo> clusters) -> alp::Result<void, io::Error> {
            ALP_TRY(const bvh::Reference leaf, writer.write_node(bvh::LeafInput{node.header(), clusters}));
            children.push_back(leaf);
            return {};
        },
        [&](const std::span<const bvh::Reference> node_children) -> alp::Result<void, io::Error> {
            const auto [left, right] = to_array<2>(node_children);

            ALP_TRY(const std::span<const Split> row, rows.pop_row());
            const Decision decision = compute_decision(row, slots, options);

            // A kept node gathers its children under one node of its own, a dissolved one hands them to its parent.
            std::vector<bvh::Reference> kept_children;
            std::vector<bvh::Reference> &destination = decision.kept ? kept_children : children;

            ALP_TRY(write_subtree(input, left, decision.left_slots, rows, writer, options, destination));
            ALP_TRY(write_subtree(input, right, decision.right_slots, rows, writer, options, destination));

            if (!decision.kept) {
                return {};
            }

            // The wide children partition the leaves the binary ones did, so the node keeps the header it had.
            ALP_TRY(const bvh::Reference kept, writer.write_node(bvh::InnerInput{node.header(), kept_children}));
            children.push_back(kept);
            return {};
        });
}

} // namespace detail

// Writes the widened tree that `splits` describes for the tree at `root` into `output`, and returns its root.
[[nodiscard]]
inline alp::Result<bvh::Reference, io::Error> write_wide_nodes(
    const SplitStorage &splits, bvh::NodeCache &input, const bvh::Reference &root,
    bvh::Storage &output, const Options &options) {
    ASSERT(options.arity >= 2 && options.arity <= MAX_ARITY, "the arity does not fit a slot count");

    return alp::run_with_context(
        alp::make_context("writing wide nodes to {}", output.directory()),
        [&] -> alp::Result<bvh::Reference, io::Error> {
            bvh::BatchWriter writer(output, options.max_batch_bytes);
            SplitReader rows(splits);

            // The root stands in one slot of nothing at all, which leaves it no way out but to keep it.
            std::vector<bvh::Reference> roots;
            ALP_TRY(detail::write_subtree(input, root, 1, rows, writer, options, roots));
            ALP_TRY(writer.flush());

            ASSERT(roots.size() == 1, "the root did not resolve to a single node");
            return roots.front();
        });
}

}
