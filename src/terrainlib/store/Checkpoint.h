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
#include <optional>
#include <utility>

#include <fmt/format.h>

#include "common_error_codes.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "store/Hash.h"
#include "store/Index.h"
#include "store/KeyTraits.h"
#include "store/Storage.h"
#include "store/index_checks.h"
#include "store/inputs.h"
#include "store/open.h"

namespace store {

template <Index IndexType, typename PayloadType>
class Checkpoint;

template <Index IndexType, typename PayloadType>
class Draft;

// Opens the draft at `draft_path` as a checkpoint for the storage at `output_path`, if it was produced from `inputs`.
template <typename Schema, Index IndexType, typename Payload, typename OpenError>
[[nodiscard]]
alp::Result<Checkpoint<IndexType, Payload>, inputs::ValidateReuseError, OpenError> open_for_reuse(
    const std::filesystem::path& draft_path,
    const std::filesystem::path& output_path,
    const typename Schema::Latest& inputs,
    alp::BasicResult<Storage<IndexType, Payload>, OpenError> (*open)(const std::filesystem::path&, Accept)
);

// The nodes of an earlier run's draft that a run may link instead of building them, or none.
template <Index IndexType, typename PayloadType>
class Checkpoint {
public:
    using Index = IndexType;
    using Payload = PayloadType;
    using Traits = typename IndexType::Traits;
    using Key = typename Traits::Key;
    using LoadError = typename Storage<IndexType, PayloadType>::LoadError;

    // A checkpoint without nodes.
    Checkpoint() = default;

    // Whether the node has a payload.
    bool has(const Key& key) const {
        assert_valid<Traits>(key);
        return this->_storage.has_value() && this->_storage->has(key);
    }

    // Reads the node's payload and checks it against its recorded hash.
    [[nodiscard]]
    alp::Result<Payload, LoadError> load(const Key& key) const {
        if (!this->_storage.has_value()) {
            return alp::make_error(NotFound{}, fmt::format("loading node {} from a checkpoint, which is empty", key));
        }
        return this->_storage->load(key);
    }

    // The node's recorded hash, without reading its files.
    [[nodiscard]]
    alp::Result<PayloadHash, NotFound, Unsupported> hash(const Key& key) const {
        if (!this->_storage.has_value()) {
            return alp::make_error(NotFound{}, fmt::format("looking up the hash of node {} in a checkpoint, which is empty", key));
        }
        return this->_storage->hash(key);
    }

private:
    // Linking a node from a checkpoint reads the files of its storage.
    template <store::Index, typename>
    friend class Draft;

    template <typename Schema, store::Index I, typename P, typename OpenError>
    friend alp::Result<Checkpoint<I, P>, inputs::ValidateReuseError, OpenError> open_for_reuse(
        const std::filesystem::path& draft_path,
        const std::filesystem::path& output_path,
        const typename Schema::Latest& inputs,
        alp::BasicResult<Storage<I, P>, OpenError> (*open)(const std::filesystem::path&, Accept)
    );

    explicit Checkpoint(Storage<IndexType, PayloadType> storage)
        : _storage(std::move(storage)) {}

    std::optional<Storage<IndexType, PayloadType>> _storage;
};

template <typename Schema, Index IndexType, typename Payload, typename OpenError>
alp::Result<Checkpoint<IndexType, Payload>, inputs::ValidateReuseError, OpenError> open_for_reuse(
    const std::filesystem::path& draft_path,
    const std::filesystem::path& output_path,
    const typename Schema::Latest& inputs,
    alp::BasicResult<Storage<IndexType, Payload>, OpenError> (*open)(const std::filesystem::path&, Accept)
) {
    ALP_TRY(store::inputs::validate_reuse<Schema>(draft_path, output_path, inputs));
    auto open_result = open(draft_path, Accept::Draft);
    ALP_TRY(auto storage, std::move(open_result));
    return Checkpoint<IndexType, Payload>(std::move(storage));
}

} // namespace store
