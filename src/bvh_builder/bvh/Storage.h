#pragma once

#include <expected>
#include <filesystem>
#include <utility>

#include <fmt/format.h>
#include <libassert/assert.hpp>

#include "storage/BaseStorage.h"
#include "bvh/Batch.h"
#include "error/Result.h"
#include "error/try.h"
#include "io/serde.h"
#include "log.h"

namespace bvh {

class Storage : private BaseStorage {
    using Base = BaseStorage;

public:
    using Batch = bvh::Batch;

    // Starts an empty storage in `directory`, which may be missing.
    // Fails when `directory` already holds something.
    [[nodiscard]]
    static alp::Result<Storage, io::FileError> create(std::filesystem::path directory) {
        ALP_TRY(Base base, Base::create(std::move(directory)));
        return Storage(std::move(base));
    }

    // Opens the storage kept in `directory`.
    // Fails when it holds no index.
    [[nodiscard]]
    static alp::Result<Storage, io::serde::Error> open(std::filesystem::path directory) {
        ALP_TRY(Base base, Base::open(std::move(directory)));
        return Storage(std::move(base));
    }

    Storage(const Storage &) = delete;
    Storage &operator=(const Storage &) = delete;

    Storage(Storage &&) noexcept = default;
    Storage &operator=(Storage &&) noexcept = default;

    using Base::directory;
    using Base::save_index;
    using Base::size;

    Batch create_batch() {
        const uint32_t index = this->allocate_index();
        return Batch(index);
    }

    [[nodiscard]]
    alp::Result<void, io::serde::Error> write(const Batch &batch) const {
        ASSERT(batch.index() < this->size());
        this->mark_dirty();
        ALP_TRY_CONTEXT(io::serde::write(batch, this->get_batch_path(batch.index())), fmt::format("writing batch {}", batch.index()));
        return {};
    }

    [[nodiscard]]
    alp::Result<Batch, io::serde::Error> read(const uint32_t index) const {
        ALP_TRY_CONTEXT(Batch batch, io::serde::read<Batch>(this->get_batch_path(index)), fmt::format("reading batch {}", index));
        ASSERT(batch.index() == index);
        return batch;
    }

private:
    explicit Storage(Base base) : Base(std::move(base)) {}
};

} // namespace bvh
