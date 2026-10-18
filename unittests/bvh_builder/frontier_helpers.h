#pragma once

#include "../result_helpers.h"

#include <cstdint>
#include <filesystem>
#include <span>
#include <vector>

#include "error/Infallible.h"
#include "error/Result.h"
#include "range_utils.h"

#include "frontier/Storage.h"
#include "frontier/layout.h"
#include "morton/utils.h"

// A saved storage in `directory` holding one batch per element of `batches`.
inline frontier::Storage make_storage(const std::filesystem::path &directory, const std::vector<std::vector<frontier::Entry>> &batches) {
    frontier::Storage storage = REQUIRE_VALUE(frontier::Storage::create(directory));
    for (const std::vector<frontier::Entry> &entries : batches) {
        REQUIRE_VALUE(storage.write(frontier::Batch{.entries = entries}));
    }
    REQUIRE_VALUE(storage.save_index());
    return storage;
}

// The entries of every batch of `storage`, in order.
inline std::vector<std::vector<frontier::Entry>> read_batches(const frontier::Storage &storage) {
    std::vector<std::vector<frontier::Entry>> result;
    for (const uint32_t index : range(storage.size())) {
        const frontier::Batch batch = REQUIRE_VALUE(storage.read(index));
        result.push_back(batch.entries);
    }
    return result;
}

// An entry sink that appends the codes it is given to `codes`, and sets `finished` once its stream ends.
struct CollectingSink {
    std::vector<morton::Code> &codes;
    bool &finished;

    alp::Result<void, alp::Infallible> push(const std::span<const frontier::Entry> entries) {
        for (const frontier::Entry &entry : entries) {
            this->codes.push_back(entry.morton);
        }
        return {};
    }

    alp::Result<void, alp::Infallible> finish() && {
        this->finished = true;
        return {};
    }
};
