#pragma once

#include <cstddef>
#include <expected>
#include <ranges>
#include <utility>

#include "io/serde.h"

#include "bvh/Storage.h"
#include "bvh/Batch.h"
#include "bvh/NodeInput.h"
#include "bvh/Reference.h"
#include "bvh/utils.h"
#include "storage/BaseBatchWriter.h"
#include "error/Result.h"
#include "error/try.h"

namespace bvh {

// Writes bvh nodes into batches with a maximum size.
class BatchWriter final : public BaseBatchWriter<Storage> {
    using Base = BaseBatchWriter<Storage>;

public:
    using Base::Base;
    using Base::compute_buffered_bytes;

    // Writes `node` to the current batch and returns its reference.
    template <NodeInput N>
    [[nodiscard]]
    alp::Result<Reference, io::serde::Error> write_node(const N &node) {
        Batch &current = this->open_batch();
        const NodeIndex node_index = current.add_node(node);
        const BatchIndex batch_index = current.index();

        ALP_TRY(this->flush_if_full());
        return bvh::Reference{
            .batch = batch_index,
            .node = node_index
        };
    }

protected:
    Batch create_batch(Storage &output) override {
        return output.create_batch();
    }

    size_t compute_buffered_bytes(const Batch &batch) const override {
        return size_bytes(batch);
    }
};

} // namespace bvh