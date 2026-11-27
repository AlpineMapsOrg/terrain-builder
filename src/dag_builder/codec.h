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
#include <string_view>
#include <vector>

#include "dag_node.h"
#include "error/Result.h"
#include "store/Codec.h"

namespace dag::codec {

// The id of both DAG codecs, which names them in a storage's metadata.
inline constexpr std::string_view id = ".dag";

// Stores a batch as its clustering in a .dag file and its metadata in a .dagmeta file.
class ClusterBatch final : public store::Codec<dag::ClusterBatch> {
public:
    std::string_view id() const override { return codec::id; }

    std::vector<std::filesystem::path> paths(const std::filesystem::path& node_path) const override;
    [[nodiscard]]
    alp::Result<dag::ClusterBatch, store::CodecReadError> read(
        const std::filesystem::path& node_path, std::optional<store::PayloadHashView> expected_hash = std::nullopt) const override;
    [[nodiscard]]
    alp::Result<store::PayloadHash, store::CodecWriteError> write(const std::filesystem::path& node_path, const dag::ClusterBatch& batch) const override;
    bool supports_hashing() const override { return true; }
};

// Reads only the metadata of a batch that ClusterBatch stored.
class Metadata final : public store::Codec<dag::NodeMetadata> {
public:
    std::string_view id() const override { return codec::id; }

    std::vector<std::filesystem::path> paths(const std::filesystem::path& node_path) const override;
    [[nodiscard]]
    alp::Result<dag::NodeMetadata, store::CodecReadError> read(
        const std::filesystem::path& node_path, std::optional<store::PayloadHashView> expected_hash = std::nullopt) const override;
    bool supports_hashing() const override { return true; }
};

// The extension of the file holding a batch's clustering.
inline constexpr std::string_view clustering_extension = ".dag";
// The extension of the file holding a batch's metadata.
inline constexpr std::string_view metadata_extension = ".dagmeta";

} // namespace dag::codec
