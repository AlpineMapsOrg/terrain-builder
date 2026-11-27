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
#include <memory>
#include <optional>
#include <ranges>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <libassert/assert.hpp>

#include "common_error_codes.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/FileError.h"
#include "io/filesystem.h"
#include "store/Codec.h"
#include "store/KeyTraits.h"
#include "store/path_layout.h"

namespace store {

// A storage's node files, read and written through its codec, without its index.
template <KeyTraits Traits, typename Payload>
class RawStorage {
public:
    using Key = typename Traits::Key;

    RawStorage(path_layout::Resolver<Key> path_resolver, std::unique_ptr<Codec<Payload>> codec)
        : _path_resolver(std::move(path_resolver))
        , _codec(std::move(codec)) {
    }

    // Reads the node's payload.
    // If an expected hash is given, it must equal the hash of the node's files.
    [[nodiscard]]
    alp::Result<Payload, CodecReadError> load(const Key& key, const std::optional<PayloadHashView> expected_hash = std::nullopt) const {
        assert_valid<Traits>(key);

        const std::filesystem::path node_path = this->_path_resolver.locate_node(key);
        const auto context = alp::make_context("reading the files of node {} at {}", key, node_path);

        ALP_TRY_CONTEXT(Payload data, this->_codec->read(node_path, expected_hash), context);
        return data;
    }

    // Writes the node's payload and returns its hash, or an empty hash if the codec does not support hashing.
    [[nodiscard]]
    alp::Result<PayloadHash, CodecWriteError> save(const Key& key, const Payload& data) const {
        assert_valid<Traits>(key);

        const std::filesystem::path node_path = this->_path_resolver.locate_node(key);
        const auto context = alp::make_context("writing the files of node {} at {}", key, node_path);

        ALP_TRY_CONTEXT(PayloadHash hash, this->_codec->write(node_path, data), context);
        return hash;
    }

    // Whether the codec returns a hash from `save` and checks an expected one in `load`.
    bool supports_hashing() const { return this->_codec->supports_hashing(); }

    // The paths of the node's files, whether or not they exist.
    std::vector<std::filesystem::path> paths(const Key& key) const {
        assert_valid<Traits>(key);
        return this->_codec->paths(this->_path_resolver.locate_node(key));
    }

    // Whether all of the node's files exist.
    [[nodiscard]]
    alp::Result<bool, io::FileError> has(const Key& key) const {
        const std::vector<std::filesystem::path> node_paths = this->paths(key);
        if (node_paths.empty()) {
            return false;
        }

        const auto context = alp::make_context("checking whether the files of node {} exist", key);
        for (const std::filesystem::path& path : node_paths) {
            ALP_TRY_CONTEXT(const bool exists, io::filesystem::exists(path), context);
            if (!exists) {
                return false;
            }
        }
        return true;
    }

    using LinkError = alp::Error<Unsupported, io::FileError>;

    // Hard-links the node's files from `source`.
    // Requires the source's files to exist and the target's not to.
    [[nodiscard]]
    alp::Result<void, LinkError> link_from(const Key& key, const RawStorage& source) const {
        assert_valid<Traits>(key);

        const std::filesystem::path source_node_path = source._path_resolver.locate_node(key);
        const std::filesystem::path target_node_path = this->_path_resolver.locate_node(key);
        const auto context = alp::make_context("hard-linking the files of node {} from {} to {}", key, source_node_path, target_node_path);

        if (this->_codec->id() != source._codec->id()) {
            const std::string finding = fmt::format("the source's codec {} differs from the target's codec {}", source._codec->id(), this->_codec->id());
            return alp::make_error(Unsupported{}, finding).context(context);
        }

        const std::vector<std::filesystem::path> source_paths = source._codec->paths(source_node_path);
        const std::vector<std::filesystem::path> target_paths = this->_codec->paths(target_node_path);
        // A codec has as many paths for every node.
        ASSERT(source_paths.size() == target_paths.size());

        for (const auto [source_path, target_path] : std::views::zip(source_paths, target_paths)) {
            ALP_TRY_CONTEXT(io::filesystem::create_parent_directories(target_path), context);
            ALP_TRY_CONTEXT(io::filesystem::create_hard_link(source_path, target_path), context);
        }
        return {};
    }

    // The storage's layout, bound to its directory.
    const path_layout::Resolver<Key>& resolver() const { return this->_path_resolver; }

    // The id of the storage's codec.
    std::string_view codec_id() const { return this->_codec->id(); }

private:
    path_layout::Resolver<Key> _path_resolver;
    std::unique_ptr<Codec<Payload>> _codec;
};

} // namespace store
