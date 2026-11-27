/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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
#include <ranges>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <fmt/std.h>
#include <libassert/assert.hpp>

#include "common_error_codes.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "store/Hash.h"
#include "store/Index.h"
#include "store/RawStorage.h"
#include "store/KeyTraits.h"
#include "store/index_checks.h"

namespace store {

template <Index IndexType, typename Payload>
class Draft;

// Read access to a storage: a published one, or a draft opened to reuse its nodes.
// All members may be called concurrently.
template <Index IndexType, typename PayloadType>
class Storage {
public:
    using Index = IndexType;
    using Payload = PayloadType;
    using Traits = typename IndexType::Traits;
    using Key = typename Traits::Key;

    // Takes the storage's node files and its index, which must be valid for the storage's codec.
    Storage(RawStorage<Traits, Payload> raw, IndexType index)
        : _raw(std::move(raw)),
          _index(std::move(index)) {
        DEBUG_ASSERT(validate_index(this->_index, this->_raw.supports_hashing()).has_value());
    }

    // Whether the node has a payload.
    bool has(const Key& key) const {
        assert_valid<Traits>(key);
        return this->_index.has(key);
    }

    using LoadError = alp::Error<NotFound, CodecReadError>;

    // Reads the node's payload and checks it against its recorded hash.
    [[nodiscard]]
    alp::Result<Payload, LoadError> load(const Key& key) const {
        const std::filesystem::path node_path = this->resolver().locate_node(key);
        const auto context = alp::make_context("loading node {} at {}", key, node_path);

        ALP_TRY_CONTEXT(detail::require_node(this->_index, key), context);

        const std::optional<PayloadHashView> expected_hash = this->_index.hash(key);
        ALP_TRY_CONTEXT(Payload data, this->_raw.load(key, expected_hash), context);
        return data;
    }

    // The paths of the node's files, whether or not they exist.
    std::vector<std::filesystem::path> paths(const Key& key) const {
        return this->_raw.paths(key);
    }

    // The node's recorded hash, without reading its files.
    // Fails with Unsupported if the storage's codec does not hash.
    [[nodiscard]]
    alp::Result<PayloadHash, NotFound, Unsupported> hash(const Key& key) const {
        assert_valid<Traits>(key);

        const std::filesystem::path node_path = this->resolver().locate_node(key);
        const auto context = alp::make_context("looking up the hash of node {} at {}", key, node_path);

        if (!this->_raw.supports_hashing()) {
            return alp::make_error(Unsupported{}, "the storage's codec does not hash").context(context);
        }

        auto hash_result = alp::to_result(this->_index.hash(key), NotFound{}, "the index records no hash for the node");
        ALP_TRY_CONTEXT(const PayloadHashView hash, std::move(hash_result), context);
        return std::ranges::to<PayloadHash>(hash);
    }

    // The storage's directory.
    const std::filesystem::path& base_path() const {
        return this->_raw.resolver().base_path();
    }

    // The storage's layout, bound to its directory.
    const path_layout::Resolver<Key>& resolver() const {
        return this->_raw.resolver();
    }

    // The id of the storage's codec.
    std::string_view codec_id() const {
        return this->_raw.codec_id();
    }

    // The storage's index.
    const IndexType& index() const {
        return this->_index;
    }

private:
    // Linking a node into a draft reads the source's node files.
    template <store::Index, typename>
    friend class Draft;

    RawStorage<Traits, Payload> _raw;
    IndexType _index;
};

} // namespace store
