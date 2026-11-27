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

#include "mesh/codec/resolve.h"

#include "io/UnknownExtension.h"
#include "mesh/codec/Gltf.h"
#include "mesh/codec/SfMesh.h"
#include "mesh/io.h"
#include "mesh/io/gltf.h"
#include "mesh/io/sf_mesh.h"
#include "string_utils.h"

namespace mesh::codec {

alp::Result<std::unique_ptr<store::Codec<mesh::Simple>>, Unsupported> resolve(const std::string_view id) {
    if (equals_ignore_ascii_case(id, mesh::io::sf_mesh::extension)) {
        return std::make_unique<SfMesh>();
    }
    auto container_result = mesh::io::gltf::container_from_extension(id);
    if (container_result.has_value()) {
        return std::make_unique<Gltf>(*container_result);
    }
    return ::io::make_unsupported_extension_error(id, mesh::io::extensions);
}

} // namespace mesh::codec
