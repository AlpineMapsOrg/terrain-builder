#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <limits>
#include <span>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <libassert/assert.hpp>

#include "io/Error.h"
#include "mem.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"

#include "containers/SegmentedBuffer.h"

#include "build/merge_cost.h"
#include "storage/BaseBatchWriter.h"
#include "storage/BatchStorage.h"
#include "bvh/NodeCache.h"
#include "bvh/Batch.h"
#include "bvh/Node.h"
#include "bvh/NodeHeader.h"
#include "bvh/Reference.h"
#include "dag/SourceId.h"
#include "widen/RowBuffer.h"
#include "widen/options.h"

namespace widen {

// What a node does when it is offered a given number of slots.
class Split {
public:
    explicit Split() : value(0) {}

    bool operator==(const Split &) const = default;

    // How many slots go to its left child.
    uint8_t left_slots() const {
        return this->value & (KEEP_BIT - 1);
    }

    // How many of `slots` go to its right child.
    uint8_t right_slots(const uint8_t slots) const {
        return slots - this->left_slots();
    }

    // Whether it becomes a node of the widened tree rather than dissolving into its parent.
    bool is_kept() const {
        return (this->value & KEEP_BIT) != 0;
    }

    // A node that is kept, giving `left_slots` of its arity to its left child.
    static Split keep(const uint8_t left_slots) {
        ASSERT(left_slots < KEEP_BIT, "the left slot count does not fit a split");
        return Split{static_cast<uint8_t>(left_slots | KEEP_BIT)};
    }

    // A node that dissolves, giving `left_slots` of the offer it was made to its left child.
    static Split dissolve(const uint8_t left_slots) {
        ASSERT(left_slots < KEEP_BIT, "the left slot count does not fit a split");
        return Split{left_slots};
    }

private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    explicit Split(const uint8_t value) : value(value) {}

    // The left slot count, with the top bit set when the node is kept.
    uint8_t value;
    static constexpr uint8_t KEEP_BIT = 1 << 7;
};

// The splits of a range of inner nodes, in post-order.
struct SplitBatch {
    // A row per inner node, holding its split at each number of slots it might be offered.
    RowBuffer<Split, uint8_t> splits;
};

using SplitStorage = BatchStorage<SplitBatch>;

// Writes split rows into batches with a maximum size.
class BatchWriter final : public BaseBatchWriter<SplitStorage> {
    using Base = BaseBatchWriter<SplitStorage>;

public:
    using Base::Base;
    using Base::compute_buffered_bytes;

    // Appends `row`, writing the batch out once it is full.
    [[nodiscard]]
    alp::Result<void, io::Error> write(const std::span<const Split> row) {
        this->open_batch().splits.push_row(row);
        return this->flush_if_full();
    }

protected:
    Batch create_batch(SplitStorage &) override {
        return Batch{};
    }

    size_t compute_buffered_bytes(const Batch &batch) const override {
        return sizeof_val(batch.splits.lengths(), batch.splits.values());
    }
};

// Reads split rows out of a storage, the last written first.
class SplitReader {
public:
    explicit SplitReader(const SplitStorage &splits) : _splits(splits), _unread_batches(splits.size()) {}

    // The row before the one popped last, valid until the next pop.
    [[nodiscard]] alp::Result<std::span<const Split>, io::Error> pop_row() {
        // A batch may hold no rows at all, so keep going back until one does.
        while (this->_row == this->_rows.splits.begin()) {
            ASSERT(this->_unread_batches > 0, "the rows ran out before the tree did");

            this->_unread_batches--;
            ALP_TRY(this->_rows, this->_splits.read(this->_unread_batches));
            this->_row = this->_rows.splits.end();
        }

        --this->_row;
        return *this->_row;
    }

private:
    const SplitStorage &_splits;
    // How many batches are left to read, the next one being the last of them.
    uint32_t _unread_batches;
    SplitBatch _rows;
    RowBuffer<Split, uint8_t>::Iterator _row = this->_rows.splits.begin();
};

namespace detail {

inline double compute_cost(std::span<const double> costs, const size_t slots) {
    if (slots > costs.size()) {
        return 0.0;
    }
    return costs[slots - 1];
}

inline std::pair<double, uint8_t> compute_best_node_split(const std::span<const double> left_costs, const std::span<const double> right_costs, const uint32_t slots) {
    DEBUG_ASSERT(slots > 1);

    double best_cost = std::numeric_limits<double>::infinity();
    uint8_t best_split = 0;
    for (uint8_t left_slots = 1; left_slots < slots; left_slots++) {
        const double left_cost = compute_cost(left_costs, left_slots);
        const double right_cost = compute_cost(right_costs, slots - left_slots);
        const double cost = left_cost + right_cost;
        if (cost < best_cost) {
            best_cost = cost;
            best_split = left_slots;
        }
    }
    return {best_cost, best_split};
}

// The buffers a walk reuses at every node.
struct Scratch {
    // A stack of cost rows, one per tabulated subtree whose parent has not consumed it yet.
    SegmentedBuffer<double> cost_stack;
    // The cost row of the node being tabulated.
    std::vector<double> costs;
    // The split row of the node being tabulated.
    std::vector<Split> row;
};

// Works out what the node with `header` does at each number of slots it might be offered, writing its cost row to `costs` and its split row to `splits`.
inline void tabulate_inner_node(
    const bvh::NodeHeader &header, const std::span<const double> left_costs, const std::span<const double> right_costs,
    const Options &options, std::vector<double> &costs, std::vector<Split> &splits) {
    // Clear scratch buffers
    costs.clear();
    splits.clear();

    const size_t length = std::min<size_t>(options.arity - 1, left_costs.size() + right_costs.size() + 1);

    // The cost of keeping the node and distributing the slots to its children.
    const double node_cost = build::compute_node_cost(header, options.error_weight);
    const auto [keep_distribute, keep_split] = compute_best_node_split(left_costs, right_costs, options.arity);
    const double keep_cost = node_cost + keep_distribute;

    // The cost of dissolving the node and distributing the slots to its children.
    double best_cost = keep_cost;
    Split best_split = Split::keep(keep_split);
    costs.push_back(best_cost);
    splits.push_back(best_split);

    for (uint8_t slots = 2; slots <= length; slots++) {
        const auto [cost, split] = compute_best_node_split(left_costs, right_costs, slots);
        if (cost < best_cost) {
            best_cost = cost;
            best_split = Split::dissolve(split);
        }
        costs.push_back(best_cost);
        splits.push_back(best_split);
    }
}

// Tabulates the subtree at `reference` post-order, writing a row per inner node and leaving the subtree's cost row on top of `scratch.cost_stack`.
[[nodiscard]]
inline alp::Result<void, io::Error> tabulate_subtree(
    bvh::NodeCache &input, const bvh::Reference &reference, const Options &options,
    BatchWriter &writer, Scratch &scratch) {
    ALP_TRY(const std::shared_ptr<const bvh::Batch> shared_batch, input.read_batch(reference.batch));
    const bvh::Batch &batch = *shared_batch;
    ASSERT(reference.node < batch.nodes().size());
    const bvh::Node node = batch.nodes()[reference.node];

    return batch.visit(
        node,
        [&](const dag::SourceId &) -> alp::Result<void, io::Error> {
            // A leaf fills one slot and costs nothing at any number of them, so its row is empty.
            scratch.cost_stack.push_empty_segment();
            return {};
        },
        [&](const std::span<const bvh::Reference> children) -> alp::Result<void, io::Error> {
            ASSERT(children.size() == 2);

            // The rows are read back to front, so walking the right child first leaves the left one first there.
            for (const bvh::Reference &child : std::views::reverse(children)) {
                ALP_TRY(tabulate_subtree(input, child, options, writer, scratch));
            }

            // Both children left their row behind, the right one first.
            const size_t cost_rows = scratch.cost_stack.segment_count();
            const std::span<const double> left_costs = scratch.cost_stack.segment(cost_rows - 1);
            const std::span<const double> right_costs = scratch.cost_stack.segment(cost_rows - 2);
            tabulate_inner_node(node.header(), left_costs, right_costs, options, scratch.costs, scratch.row);

            // Every subtree leaves one row, so the node's replaces the two its children left.
            scratch.cost_stack.pop_segments(2);
            scratch.cost_stack.push_new_segment(scratch.costs);

            return writer.write(scratch.row);
        });
}

} // namespace detail

// Fills `splits` with what every node of the tree at `root` does at each number of slots it might be offered.
[[nodiscard]]
inline alp::Result<void, io::Error> tabulate_splits(bvh::NodeCache &input, const bvh::Reference &root, SplitStorage &splits, const Options &options) {
    ASSERT(options.arity >= 2 && options.arity <= MAX_ARITY);

    return alp::run_with_context(
        alp::make_context("tabulating splits for a tree at batch {} node {}", root.batch, root.node),
        [&] -> alp::Result<void, io::Error> {
            BatchWriter writer(splits, options.max_batch_bytes);

            detail::Scratch scratch;
            ALP_TRY(detail::tabulate_subtree(input, root, options, writer, scratch));

            return writer.flush();
        });
}

}
