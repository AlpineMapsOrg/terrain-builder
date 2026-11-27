#pragma once

#include <atomic>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <utility>

#include <fmt/format.h>

#include "error/Result.h"
#include "error/format.h"
#include "error/log.h"
#include "error/try.h"
#include "io/serde.h"
#include "io/FileError.h"
#include "io/filesystem.h"
#include "io/filesystem_utils.h"
#include "log.h"

class BaseStorage {
public:
    struct Index {
        uint32_t batch_count = 0;
    };

    // Starts an empty storage in `directory`, which may be missing.
    // Fails when `directory` already holds something.
    [[nodiscard]]
    static alp::Result<BaseStorage, io::FileError> create(std::filesystem::path directory) {
        ALP_TRY(io::filesystem::create_empty_directory(directory));
        return BaseStorage(std::move(directory));
    }

    // Opens the storage kept in `directory`.
    // Fails when it holds no index.
    [[nodiscard]]
    static alp::Result<BaseStorage, io::serde::Error> open(std::filesystem::path directory) {
        BaseStorage storage(std::move(directory));
        ALP_TRY_CONTEXT(const Index index, io::serde::read<Index>(storage.get_index_path()), fmt::format("opening a storage at {}", storage._directory));
        storage._next.store(index.batch_count);
        return storage;
    }

    BaseStorage(const BaseStorage &) = delete;
    BaseStorage &operator=(const BaseStorage &) = delete;

    BaseStorage(BaseStorage &&other) noexcept
        : _directory(std::move(other._directory)),
          _next(other._next.load()),
          _dirty(other._dirty.exchange(false)) {}

    BaseStorage &operator=(BaseStorage &&other) noexcept {
        this->_directory = std::move(other._directory);
        this->_next.store(other._next.load());
        this->_dirty.store(other._dirty.exchange(false));
        return *this;
    }

    ~BaseStorage() {
        if (!this->_dirty.load()) {
            return;
        }

        LOG_WARN("Index was not saved upon storage destruction, use save_index().");

        LOG_IF_ERROR(this->save_index(), "closing a storage at {}", this->_directory);
    }

    [[nodiscard]]
    alp::Result<void, io::serde::Error> save_index() const {
        const Index index{
            .batch_count = this->_next.load(),
        };

        ALP_TRY_CONTEXT(io::serde::write(index, this->get_index_path(), io::serde::WriteOptions{.mode = io::WriteMode::Replace}), fmt::format("saving an index to {}", this->get_index_path()));
        this->_dirty = false;
        return {};
    }

    uint32_t size() const {
        return this->_next.load();
    }

    // The directory the batches and the index are stored in.
    const std::filesystem::path &directory() const {
        return this->_directory;
    }

    // Deletes the storage's directory and everything it wrote.
    [[nodiscard]]
    alp::Result<void, io::FileError> remove() && {
        ALP_TRY(io::filesystem::remove_all(this->_directory));

        // Writing the index back on destruction would recreate the directory.
        this->_next.store(0);
        this->_dirty = false;
        return {};
    }

protected:
    uint32_t allocate_index() {
        return this->_next.fetch_add(1);
    }

    void mark_dirty() const {
        this->_dirty = true;
    }

    std::filesystem::path get_batch_path(const uint32_t index) const {
        return this->_directory / fmt::format("batch-{:08}.bin", index);
    }

private:
    explicit BaseStorage(std::filesystem::path directory) : _directory(std::move(directory)) {}

    std::filesystem::path get_index_path() const {
        return this->_directory / "index.bin";
    }

    std::filesystem::path _directory;
    std::atomic<uint32_t> _next = 0;
    mutable std::atomic<bool> _dirty = false;
};

