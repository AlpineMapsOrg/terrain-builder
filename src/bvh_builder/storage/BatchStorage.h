#pragma once

#include <cstdint>
#include <expected>
#include <filesystem>
#include <utility>

#include <fmt/format.h>
#include <libassert/assert.hpp>

#include "storage/BaseStorage.h"
#include "io/serde.h"
#include "error/Result.h"
#include "error/try.h"

// A run of batches, each written whole and read back by the index it was written at.
template <typename BatchType>
class BatchStorage : private BaseStorage {
    using Base = BaseStorage;

public:
    using Batch = BatchType;

    // Starts an empty storage in `directory`, which may be missing.
    // Fails when `directory` already holds something.
    [[nodiscard]]
    static alp::Result<BatchStorage, io::FileError> create(std::filesystem::path directory) {
        ALP_TRY(Base base, Base::create(std::move(directory)));
        return BatchStorage(std::move(base));
    }

    // Opens the storage kept in `directory`.
    // Fails when it holds no index.
    [[nodiscard]]
    static alp::Result<BatchStorage, io::serde::Error> open(std::filesystem::path directory) {
        ALP_TRY(Base base, Base::open(std::move(directory)));
        return BatchStorage(std::move(base));
    }

    BatchStorage(const BatchStorage &) = delete;
    BatchStorage &operator=(const BatchStorage &) = delete;

    BatchStorage(BatchStorage &&) noexcept = default;
    BatchStorage &operator=(BatchStorage &&) noexcept = default;

    using Base::directory;
    using Base::remove;
    using Base::save_index;
    using Base::size;

    [[nodiscard]]
    alp::Result<uint32_t, io::serde::Error> write(const Batch &batch) {
        const uint32_t index = this->allocate_index();
        ALP_TRY(this->write(index, batch));
        this->mark_dirty();
        return index;
    }

    [[nodiscard]]
    alp::Result<uint32_t, io::serde::Error> write(const uint32_t index, const Batch &batch) {
        ASSERT(index < this->size());
        ALP_TRY_CONTEXT(io::serde::write(batch, this->get_batch_path(index)), fmt::format("writing batch {}", index));
        return index;
    }

    [[nodiscard]]
    alp::Result<Batch, io::serde::Error> read(const uint32_t index) const {
        ASSERT(index < this->size());
        ALP_TRY_CONTEXT(Batch batch, io::serde::read<Batch>(this->get_batch_path(index)), fmt::format("reading batch {}", index));
        return batch;
    }

private:
    explicit BatchStorage(Base base) : Base(std::move(base)) {}
};
