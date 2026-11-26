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

#include "mesh/io/sf_mesh.h"

#include <utility>

#include <fmt/format.h>
#include <fmt/std.h>
#include <libassert/assert.hpp>

#include "error/try.h"
#include "mesh/EncodedMesh.h"

namespace mesh::io::sf_mesh {

namespace {

    alp::Result<Payload, mesh::EncodeError> encode_payload(const mesh::Simple& mesh, const mesh::EncodeOptions options) {
        ALP_TRY(mesh::Encoded encoded, mesh::encode(mesh, options));
        return Payload {
            .vertex_count = encoded.header.vertex_count,
            .face_count = encoded.header.face_count,
            .triangles = std::move(encoded.triangles),
            .positions = std::move(encoded.positions),
            .uvs = std::move(encoded.uvs),
            .texture = std::move(encoded.texture),
        };
    }

    alp::Result<mesh::Simple, mesh::DecodeError> decode_payload(Payload payload) {
        const mesh::Encoded encoded{
            .header = {
                .version = 1,
                .n_dims = mesh::Simple::n_dims,
                .component_type = mesh::component_type_id<mesh::Simple::Component>(),
                .vertex_count = payload.vertex_count,
                .face_count = payload.face_count,
            },
            .triangles = std::move(payload.triangles),
            .positions = std::move(payload.positions),
            .uvs = std::move(payload.uvs),
            .texture = std::move(payload.texture),
        };
        ALP_TRY(mesh::Simple mesh, mesh::decode(encoded));
        return mesh;
    }

} // namespace

alp::Result<mesh::Simple, DeserializeError> deserialize(const ::io::bytes::span bytes, const ::io::envelope::DeserializeOptions options) {
    ALP_TRY(Payload payload, ::io::envelope::deserialize<Schema>(bytes, options));
    ALP_TRY(validate(payload));
    ALP_TRY(mesh::Simple mesh, decode_payload(std::move(payload)));
    return mesh;
}

alp::Result<::io::bytes::vector, SerializeError> serialize(const mesh::Simple& mesh, const mesh::EncodeOptions options) {
    ALP_TRY(const Payload payload, encode_payload(mesh, options));
    DEBUG_ASSERT(validate(payload).has_value());
    ALP_TRY(::io::bytes::vector bytes, ::io::envelope::serialize<Schema>(payload));
    return bytes;
}

alp::Result<mesh::Simple, LoadError> load_from_path(const std::filesystem::path& path, const ::io::envelope::ReadOptions options) {
    ALP_TRY(const ::io::bytes::vector bytes, ::io::bytes::read(path));
    ALP_TRY_CONTEXT(mesh::Simple mesh, deserialize(bytes, options), fmt::format("decoding the SF mesh read from {}", path));
    return mesh;
}

alp::Result<::io::bytes::vector, SaveError> save_to_path(const mesh::Simple& mesh, const std::filesystem::path& path, const SaveOptions& options) {
    const mesh::EncodeOptions encode_options{
        .texture_format = options.texture_format,
    };
    ALP_TRY_CONTEXT(const Payload payload, encode_payload(mesh, encode_options), fmt::format("encoding the SF mesh for {}", path));
    DEBUG_ASSERT(validate(payload).has_value());

    const ::io::envelope::WriteOptions write_options{
        .mode = options.mode,
        .make_dirs = options.make_dirs,
    };
    ALP_TRY(::io::bytes::vector hash, ::io::envelope::write<Schema>(payload, path, write_options));
    return hash;
}

} // namespace mesh::io::sf_mesh
