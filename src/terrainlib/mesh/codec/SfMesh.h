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
#include <utility>
#include <vector>

#include "error/Error.h"
#include "error/Result.h"
#include "error/try.h"
#include "io/OpenCvError.h"
#include "io/WriteMode.h"
#include "io/envelope.h"
#include "io/mat.h"
#include "mesh/SimpleMesh.h"
#include "mesh/encode_error.h"
#include "mesh/io/options.h"
#include "mesh/io/sf_mesh.h"
#include "path_utils.h"
#include "store/Codec.h"
#include "store/envelope_utils.h"

namespace mesh::codec {

namespace detail::sf_mesh {

    // The failure reading a node for a failure loading its SF mesh file, which it keeps as its source.
    [[nodiscard]]
    inline store::CodecReadError to_codec_read_error(mesh::io::sf_mesh::LoadError error) {
        return std::move(error)
            .map<::io::envelope::ReadError>(store::to_codec_read_error)
            .map<mesh::io::sf_mesh::CorruptData, mesh::GeometryDecodeError, ::io::mat::CorruptData>(store::CorruptData{})
            .map<::io::OpenCvError>(store::Unknown{});
    }

    // The failure writing a node for a failure saving its SF mesh file, which it keeps as its source.
    [[nodiscard]]
    inline store::CodecWriteError to_codec_write_error(mesh::io::sf_mesh::SaveError error) {
        return std::move(error)
            .map<::io::envelope::WriteError>(store::to_codec_write_error)
            .map<::io::OpenCvError>(store::Unknown{});
    }

} // namespace detail::sf_mesh

class SfMesh final : public store::Codec<mesh::Simple> {
public:
    std::string_view id() const override { return mesh::io::sf_mesh::extension; }

    std::vector<std::filesystem::path> paths(const std::filesystem::path& node_path) const override { return { this->file_path(node_path) }; }

    [[nodiscard]]
    alp::Result<mesh::Simple, store::CodecReadError> read(const std::filesystem::path& node_path, const std::optional<store::PayloadHashView> expected_hash = std::nullopt) const override {
        const std::filesystem::path path = this->file_path(node_path);

        auto load_result = mesh::io::sf_mesh::load_from_path(path, {.expected_hash = expected_hash});
        ALP_TRY(mesh::Simple mesh, std::move(load_result).transform_error(detail::sf_mesh::to_codec_read_error));

        return mesh;
    }

    [[nodiscard]]
    alp::Result<store::PayloadHash, store::CodecWriteError> write(const std::filesystem::path& node_path, const mesh::Simple& mesh) const override {
        const std::filesystem::path path = this->file_path(node_path);

        const mesh::io::SaveOptions options{
            .mode = ::io::WriteMode::CreateNew,
        };
        auto save_result = mesh::io::sf_mesh::save_to_path(mesh, path, options);
        ALP_TRY(store::PayloadHash hash, std::move(save_result).transform_error(detail::sf_mesh::to_codec_write_error));

        return hash;
    }

    bool supports_hashing() const override { return true; }

private:
    // The file that holds the node at `node_path`.
    std::filesystem::path file_path(const std::filesystem::path& node_path) const { return paths::with_added_extension(node_path, mesh::io::sf_mesh::extension); }
};

} // namespace mesh::codec
