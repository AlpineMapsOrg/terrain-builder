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
#include <optional>
#include <vector>

#include "io/envelope.h"
#include "mesh/SimpleMesh.h"
#include "mesh/codec/SfMeshFormat.h"
#include "store/Codec.h"

namespace mesh::codec {

class SfMesh final : public store::Codec<mesh::Simple> {
public:
    std::vector<std::filesystem::path> paths(const std::filesystem::path& node_path) const override { return { add_extension(node_path, ".sfmesh") }; }

    Expected<mesh::Simple> read(
        const std::filesystem::path& node_path, const std::optional<store::NodeDataHashView> expected_hash = std::nullopt) const override
    {
        const std::filesystem::path path = paths(node_path).front();
        auto payload = ::io::envelope::read<mesh::sf::Schema>(path, expected_hash);
        if (!payload) {
            return Error::propagate(std::move(payload), "read SF mesh");
        }
        if (auto valid = mesh::sf::validate(*payload); !valid) {
            return Error::propagate(std::move(valid), Error::Code::CorruptData, "validate SF mesh read from \"" + path.string() + "\"");
        }
        auto decoded = mesh::sf::decode_payload(std::move(*payload));
        if (!decoded) {
            return Error::propagate(std::move(decoded), "decode SF mesh \"" + path.string() + "\"");
        }
        return std::move(*decoded);
    }

    Expected<store::NodeDataHash> write(const std::filesystem::path& node_path, const mesh::Simple& mesh) const override
    {
        return write(node_path, mesh, {}, ::io::byte::WriteMode::CreateNew);
    }

    Expected<store::NodeDataHash> write(
        const std::filesystem::path& node_path, const mesh::Simple& mesh, const mesh::EncodeOptions options, const ::io::byte::WriteMode mode) const
    {
        const std::filesystem::path path = paths(node_path).front();
        auto payload = mesh::sf::encode_payload(mesh, options);
        if (!payload) {
            return Error::propagate(std::move(payload), "encode SF mesh \"" + path.string() + "\"");
        }
        if (auto valid = mesh::sf::validate(*payload); !valid) {
            return Error::propagate(std::move(valid), "validate SF mesh for \"" + path.string() + "\"");
        }
        auto hash = ::io::envelope::write<mesh::sf::Schema>(*payload, path, mode);
        if (!hash) {
            return Error::propagate(std::move(hash), "write SF mesh");
        }
        return std::move(*hash);
    }

    bool supports_hashing() const override { return true; }
};

} // namespace mesh::codec
