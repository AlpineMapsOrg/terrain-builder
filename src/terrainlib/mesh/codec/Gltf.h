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
#include "io/mat.h"
#include "mesh/SimpleMesh.h"
#include "mesh/io/gltf.h"
#include "mesh/io/options.h"
#include "path_utils.h"
#include "store/Codec.h"

namespace mesh::codec {

namespace detail::gltf {

    // The failure reading a node for a failure loading its glTF file, which it keeps as its source.
    [[nodiscard]]
    inline store::CodecReadError to_codec_read_error(mesh::io::gltf::LoadError error) {
        return std::move(error)
            .map<mesh::io::gltf::CorruptData, ::io::mat::CorruptData>(store::CorruptData{})
            .map<::io::OpenCvError>(store::Unknown{});
    }

    // The failure writing a node for a failure saving its glTF file, which it keeps as its source.
    [[nodiscard]]
    inline store::CodecWriteError to_codec_write_error(mesh::io::gltf::SaveError error) {
        return std::move(error).map<::io::OpenCvError>(store::Unknown{});
    }

} // namespace detail::gltf

class Gltf final : public store::Codec<mesh::Simple> {
public:
    explicit Gltf(const mesh::io::gltf::Container container) : _container(container) {
    }

    std::string_view id() const override {
        return mesh::io::gltf::to_extension(this->_container);
    }

    std::vector<std::filesystem::path> paths(const std::filesystem::path& node_path) const override {
        return { this->file_path(node_path) };
    }

    [[nodiscard]]
    alp::Result<mesh::Simple, store::CodecReadError> read(const std::filesystem::path& node_path, std::optional<store::PayloadHashView> /* expected_hash */ = std::nullopt) const override {
        const std::filesystem::path path = this->file_path(node_path);

        auto load_result = mesh::io::gltf::load_from_path(path);
        ALP_TRY(mesh::Simple mesh, std::move(load_result).transform_error(detail::gltf::to_codec_read_error));

        return mesh;
    }

    [[nodiscard]]
    alp::Result<store::PayloadHash, store::CodecWriteError> write(const std::filesystem::path& node_path, const mesh::Simple& mesh) const override {
        const std::filesystem::path path = this->file_path(node_path);

        const mesh::io::SaveOptions options{
            .mode = ::io::WriteMode::CreateNew,
        };
        auto save_result = mesh::io::gltf::save_to_path(mesh, path, options);
        ALP_TRY(std::move(save_result).transform_error(detail::gltf::to_codec_write_error));

        return store::PayloadHash{};
    }

    mesh::io::gltf::Container container() const {
        return this->_container;
    }

private:
    // The file that holds the node at `node_path`.
    std::filesystem::path file_path(const std::filesystem::path& node_path) const {
        return paths::with_added_extension(node_path, mesh::io::gltf::to_extension(this->_container));
    }

    mesh::io::gltf::Container _container;
};

} // namespace mesh::codec
