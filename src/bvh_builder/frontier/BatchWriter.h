#pragma once

#include <cstddef>
#include <expected>
#include <span>

#include "error/Result.h"
#include "io/serde.h"

#include "frontier/Storage.h"
#include "frontier/layout.h"
#include "mem.h"
#include "storage/BaseBatchWriter.h"

namespace frontier {

// Writes frontier entries into batches with a maximum size.
class BatchWriter final : public BaseBatchWriter<Storage> {
    using Base = BaseBatchWriter<Storage>;

public:
    using Base::Base;
    using Base::compute_buffered_bytes;

    // Appends `entry`, writing the batch out once it is full.
    [[nodiscard]]
    alp::Result<void, io::serde::Error> write(const Entry &entry) {
        this->open_batch().entries.push_back(entry);
        return this->flush_if_full();
    }

protected:
    Batch create_batch(Storage &) override {
        return Batch{};
    }

    size_t compute_buffered_bytes(const Batch &batch) const override {
        return sizeof_val(std::span{batch.entries});
    }
};

} // namespace frontier