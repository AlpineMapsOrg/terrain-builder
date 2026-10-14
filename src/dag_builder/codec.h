/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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
#include <string_view>
#include <vector>

#include "dag_node.h"
#include "store/Codec.h"

namespace dag::codec {

// A node's identity is the sixteen-byte concatenation of the .dag and .dagmeta envelope hashes,
// so that each file can be verified on its own.
class ClusterBatch final : public store::Codec<dag::ClusterBatch> {
public:
    std::vector<std::filesystem::path> paths(const std::filesystem::path& node_path) const override;
    Expected<dag::ClusterBatch> read(
        const std::filesystem::path& node_path, std::optional<store::NodeDataHashView> expected_hash = std::nullopt) const override;
    Expected<store::NodeDataHash> write(const std::filesystem::path& node_path, const dag::ClusterBatch& batch) const override;
    bool supports_hashing() const override { return true; }
};

// Reads only the .dagmeta file and verifies it against the second half of an expected node identity.
class Metadata final : public store::Codec<dag::NodeMetadata> {
public:
    std::vector<std::filesystem::path> paths(const std::filesystem::path& node_path) const override;
    Expected<dag::NodeMetadata> read(
        const std::filesystem::path& node_path, std::optional<store::NodeDataHashView> expected_hash = std::nullopt) const override;
    bool supports_hashing() const override { return true; }
};

Expected<std::unique_ptr<store::Codec<dag::ClusterBatch>>> cluster_batch_from_extension(std::string_view extension);
Expected<std::unique_ptr<store::Codec<dag::NodeMetadata>>> metadata_from_extension(std::string_view extension);

} // namespace dag::codec
