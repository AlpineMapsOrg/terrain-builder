/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Martin Braunsperger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#pragma once

#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <ranges>
#include <shared_mutex>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <fmt/std.h>
#include <libassert/assert.hpp>

#include "error/MessageCode.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/FileError.h"
#include "store/Checkpoint.h"
#include "store/Codec.h"
#include "store/Hash.h"
#include "store/Index.h"
#include "store/IndexFormat.h"
#include "store/KeyTraits.h"
#include "store/PersistedIndex.h"
#include "store/RawStorage.h"
#include "store/Storage.h"
#include "store/View.h"
#include "store/index_checks.h"
#include "store/path_layout.h"
#include "store/storage_paths.h"

namespace store {

using CreateError = alp::Error<InvalidStoragePath, io::FileError, WriteIndexError>;
using PublishError = alp::Error<WriteIndexError, io::FileError>;

template <Index IndexType, typename Payload>
class Draft;

template <Index IndexType, typename Payload>
alp::Result<Draft<IndexType, Payload>, CreateError> create(
    const std::filesystem::path& final_path,
    IndexFormat<IndexType> format,
    path_layout::Layout<typename IndexType::Traits::Key> layout,
    std::unique_ptr<Codec<Payload>> codec
);

template <Index IndexType, typename Payload>
alp::Result<void, PublishError> publish(Draft<IndexType, Payload> draft);

// A storage being written, at `<path>.part`.
// This class is thread-safe.
template <Index IndexType, typename PayloadType>
class Draft {
public:
    using Index = IndexType;
    using Payload = PayloadType;
    using Traits = typename IndexType::Traits;
    using Key = typename Traits::Key;
    using LoadError = typename Storage<IndexType, Payload>::LoadError;
    using SaveError = alp::Error<AlreadyExists, CodecWriteError>;
    using LinkError = alp::Error<AlreadyExists, NotFound, typename RawStorage<Traits, Payload>::LinkError>;

    // Whether the node has a payload.
    bool has(const Key& key) const {
        assert_valid<Traits>(key);
        std::shared_lock lock(*this->_mutex);
        return this->_index.index().has(key);
    }

    // Reads the node's payload and checks it against its recorded hash.
    [[nodiscard]]
    alp::Result<Payload, LoadError> load(const Key& key) const {
        const std::filesystem::path node_path = this->_raw.resolver().locate_node(key);
        const auto context = alp::make_context("loading node {} at {}", key, node_path);

        std::shared_lock lock(*this->_mutex);

        ALP_TRY_CONTEXT(detail::require_node(this->_index.index(), key), context);

        const std::optional<PayloadHashView> expected_hash = this->_index.index().hash(key);
        ALP_TRY_CONTEXT(Payload data, this->_raw.load(key, expected_hash), context);
        return data;
    }

    // The node's recorded hash, without reading its files.
    // Fails with Unsupported if the storage's codec does not hash.
    [[nodiscard]]
    alp::Result<PayloadHash, NotFound, Unsupported> hash(const Key& key) const {
        assert_valid<Traits>(key);

        const std::filesystem::path node_path = this->_raw.resolver().locate_node(key);
        const auto context = alp::make_context("looking up the hash of node {} at {}", key, node_path);

        if (!this->_raw.supports_hashing()) {
            return alp::make_error(Unsupported{}, "the storage's codec does not hash").context(context);
        }

        std::shared_lock lock(*this->_mutex);

        auto hash_result = alp::to_result(this->_index.index().hash(key), NotFound{}, "the index records no hash for the node");
        ALP_TRY_CONTEXT(const PayloadHashView hash, std::move(hash_result), context);
        return std::ranges::to<PayloadHash>(hash);
    }

    // A copy of the index as it is at the time of the call.
    IndexType index() const {
        std::shared_lock lock(*this->_mutex);
        return this->_index.index();
    }

    // The draft's directory, `<path>.part`.
    const std::filesystem::path& base_path() const {
        return this->_raw.resolver().base_path();
    }

    // The paths of the node's files, whether or not they exist.
    std::vector<std::filesystem::path> paths(const Key& key) const {
        return this->_raw.paths(key);
    }

    // The storage's layout, bound to its directory.
    const path_layout::Resolver<Key>& resolver() const {
        return this->_raw.resolver();
    }

    // The id of the storage's codec.
    std::string_view codec_id() const {
        return this->_raw.codec_id();
    }

    // Writes the node's payload and records the node with its hash.
    [[nodiscard]]
    alp::Result<void, SaveError> save(const Key& key, const Payload& payload) {
        const std::filesystem::path node_path = this->_raw.resolver().locate_node(key);
        const auto context = alp::make_context("saving node {} at {}", key, node_path);

        {
            std::shared_lock lock(*this->_mutex);
            ALP_TRY_CONTEXT(detail::require_no_node(this->_index.index(), key), context);
        }

        ALP_TRY_CONTEXT(PayloadHash hash, this->write_payload(key, payload), context);

        {
            std::unique_lock lock(*this->_mutex);
            ALP_TRY_CONTEXT(this->commit(key, std::move(hash)), context);
        }
        return {};
    }

    // Hard-links the node's files from `source` and records the node with the source's hash.
    template <View S>
        requires std::same_as<typename S::Index, IndexType> && std::same_as<typename S::Payload, Payload>
    [[nodiscard]]
    alp::Result<void, LinkError> link_from(const Key& key, const S& source) {
        const std::filesystem::path target_node_path = this->_raw.resolver().locate_node(key);
        const std::filesystem::path source_node_path = source.resolver().locate_node(key);
        const auto target_context = alp::make_context("checking whether target node {} exists at {} before linking", key, target_node_path);
        const auto source_context = alp::make_context("checking whether source node {} exists at {} before linking", key, source_node_path);
        const auto context = alp::make_context("linking node {} from {} to {}", key, source_node_path, target_node_path);

        // The source is asked before the lock is taken, so that a draft linking from itself fails instead of waiting for its own lock.
        if (!source.has(key)) {
            return alp::make_error(NotFound{}, "the source does not have the node").context(source_context);
        }
        // A codec that does not hash records no hash, which commit drops.
        PayloadHash source_hash = source.hash(key).value_or(PayloadHash{});

        std::unique_lock lock(*this->_mutex);

        ALP_TRY_CONTEXT(detail::require_no_node(this->_index.index(), key), target_context);

        alp::Result<void, LinkError> link_result = this->_raw.link_from(key, source._raw);
        link_result = alp::map_code(std::move(link_result), std::errc::file_exists, AlreadyExists{});
        ALP_TRY_CONTEXT(std::move(link_result), context);

        ALP_TRY_CONTEXT(this->commit(key, std::move(source_hash)), context);
        return {};
    }

    // Hard-links the node's files from the checkpoint and records the node with the checkpoint's hash.
    [[nodiscard]]
    alp::Result<void, LinkError> link_from(const Key& key, const Checkpoint<IndexType, Payload>& checkpoint) {
        if (!checkpoint._storage.has_value()) {
            return alp::make_error(NotFound{}, fmt::format("linking node {} from a checkpoint, which is empty", key));
        }
        return this->link_from(key, *checkpoint._storage);
    }

    // Writes the index into the draft's directory if it has unsaved changes.
    [[nodiscard]]
    alp::Result<void, WriteIndexError> save_index() {
        std::unique_lock lock(*this->_mutex);
        return this->_index.save();
    }

private:
    template <store::Index I, typename P>
    friend alp::Result<Draft<I, P>, CreateError> create(
        const std::filesystem::path& final_path,
        IndexFormat<I> format,
        path_layout::Layout<typename I::Traits::Key> layout,
        std::unique_ptr<Codec<P>> codec
    );

    template <store::Index I, typename P>
    friend alp::Result<void, PublishError> publish(Draft<I, P> draft);

    Draft(RawStorage<Traits, Payload> raw, PersistedIndex<IndexType> index)
        : _raw(std::move(raw)),
          _index(std::move(index)),
          _mutex(std::make_unique<std::shared_mutex>()) {}

    // Writes the node's files and returns the hash the codec gives for them.
    [[nodiscard]]
    alp::Result<PayloadHash, SaveError> write_payload(const Key& key, const Payload& payload) const {
        alp::Result<PayloadHash, SaveError> save_result = this->_raw.save(key, payload);
        save_result = alp::map_code(std::move(save_result), std::errc::file_exists, AlreadyExists{});
        ALP_TRY(PayloadHash hash, std::move(save_result));
        return hash;
    }

    // Records the node with `hash`, which is dropped if the codec does not hash.
    // Requires the exclusive lock.
    // Fails with AlreadyExists if the index already lists the node.
    [[nodiscard]]
    alp::Result<void, AlreadyExists> commit(const Key& key, PayloadHash hash) {
        const auto context = alp::make_context("committing node {}", key);

        ALP_TRY_CONTEXT(detail::require_no_node(this->_index.index(), key), context);

        std::optional<PayloadHash> recorded_hash;
        if (this->_raw.supports_hashing()) {
            recorded_hash = std::move(hash);
        }
        const bool added = this->_index.add(key, std::move(recorded_hash));
        // The exclusive lock has been held since the check above.
        ASSERT(added);
        return {};
    }

    RawStorage<Traits, Payload> _raw;
    PersistedIndex<IndexType> _index;
    std::unique_ptr<std::shared_mutex> _mutex;
};

} // namespace store
