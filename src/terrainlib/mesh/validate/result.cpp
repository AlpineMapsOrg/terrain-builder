/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2025 Martin Braunsperger
 * Copyright (C) 2025 Adam Celarek-Litofcenko
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

#include <cstddef>
#include <cstdint>
#include <span>
#include <unordered_map>
#include <vector>

#include <fmt/format.h>
#include <glm/glm.hpp>
#include <glm/gtx/hash.hpp>

#include "log.h"
#include "enumerate.h"
#include "error/try.h"
#include "error/utils.h"
#include "mesh/cleanup.h"
#include "mesh/connectivity/connected_components.h"
#include "mesh/convert.h"
#include "mesh/geometry.h"
#include "mesh/connectivity/manifold.h"
#include "mesh/validate/result.h"
#include "mesh/SimpleMesh.h"
#include "mesh/View.h"

namespace mesh {
namespace {
// inline constexpr double EPSILON = 1e-12;

constexpr bool has_flag(const ValidationFlags set, const ValidationFlags flag) noexcept {
    return (set & flag) != ValidationFlags::None;
}

// inline bool has_duplicate_faces(const std::span<const glm::uvec3> triangles, const bool ignore_orientation = true) {
//     std::unordered_map<glm::uvec3, uint32_t> counts;
//     counts.reserve(triangles.size());
//
//     for (const glm::uvec3 &triangle : triangles) {
//         const glm::uvec3 normalized = normalize_triangle(triangle, !ignore_orientation);
//         counts[normalized] += 1;
//     }
//
//     for (const auto &[_triangle, count] : counts) {
//         if (count > 1) {
//             return true;
//         }
//     }
//
//     return false;
// }

template <glm::length_t n_dims, typename T>
alp::Result<void, InvalidMesh> validate_impl_basic(const mesh::View_<n_dims, T> &mesh) {
    using Mesh = mesh::View_<n_dims, T>;
    using Uv = typename Mesh::Uv;

    static_assert(n_dims == 2 || n_dims == 3, "Mesh must be 2D or 3D");

    if (mesh.has_uvs() && mesh.positions.size() != mesh.uvs.size()) {
        return alp::make_error(InvalidMesh{}, fmt::format("the mesh has {} UVs for {} positions", mesh.uvs.size(), mesh.positions.size()));
    }

    for (const auto [uv_index, uv] : enumerate(mesh.uvs)) {
        for (uint8_t k = 0; k < 2; k++) {
            if (!(uv[k] >= static_cast<typename Uv::value_type>(0) && uv[k] <= static_cast<typename Uv::value_type>(1))) {
                return alp::make_error(InvalidMesh{}, fmt::format("UV {} lies outside [0, 1]", uv_index));
            }
        }
    }

    const size_t vertex_count = mesh.vertex_count();
    for (const auto [triangle_index, triangle] : enumerate(mesh.triangles)) {
        for (uint8_t k = 0; k < 3; k++) {
            const size_t vertex_index = triangle[k];
            if (vertex_index >= vertex_count) {
                return alp::make_error(InvalidMesh{}, fmt::format("triangle {} names vertex {}, but there are {} vertices", triangle_index, vertex_index, vertex_count));
            }
        }
    }

    for (const auto [triangle_index, triangle] : enumerate(mesh.triangles)) {
        if (is_degenerate(triangle)) {
            return alp::make_error(InvalidMesh{}, fmt::format("triangle {} is degenerate", triangle_index));
        }
    }

    return {};
}

template <glm::length_t n_dims, typename T>
alp::Result<void, InvalidMesh> validate_impl_topology(const mesh::View_<n_dims, T> &mesh, const ValidationFlags flags) {
    if (has_flag(flags, ValidationFlags::SingleComponent)) {
        if (!is_single_component(mesh)) {
            return alp::make_error(InvalidMesh{}, "the mesh has more than one connected component");
        }
    }

    if (has_flag(flags, ValidationFlags::Manifold)) {
        if (!is_manifold(mesh)) {
            return alp::make_error(InvalidMesh{}, "the mesh is not manifold");
        }
    }

    return {};
}

template <glm::length_t n_dims, typename T>
alp::Result<void, InvalidMesh> validate_impl_geometry(const mesh::View_<n_dims, T> &mesh) {
    static_assert(n_dims >= 2, "Geometry checks require n_dims >= 2");

    const std::vector<uint32_t> isolated_vertices = find_isolated_vertices(mesh);
    if (!isolated_vertices.empty()) {
        return alp::make_error(InvalidMesh{}, fmt::format("vertex {} belongs to no triangle", isolated_vertices.front()));
    }

    /*
    While this check makes sense on its own
    This produces false positives when validating the geometry for uv unwrapping without uvs
    for (const glm::uvec3 &triangle : mesh.triangles) {
        const bool is_empty = is_empty_triangle(triangle, mesh.positions);
        if (mesh.has_uvs()) {
            DEBUG_ASSERT(!is_empty_triangle(triangle, mesh.uvs));
        } else {
            DEBUG_ASSERT(is_empty);
        }
    }
    */

    return {};
}
}

namespace detail {

template <glm::length_t n_dims, typename T>
alp::Result<void, InvalidMesh> validate_impl(const mesh::View_<n_dims, T> &mesh, const ValidationFlags flags) {
    if (has_flag(flags, ValidationFlags::Basic)) {
        ALP_TRY(validate_impl_basic(mesh));
    }

    ALP_TRY(validate_impl_topology(mesh, flags));

    if (has_flag(flags, ValidationFlags::Geometry)) {
        ALP_TRY(validate_impl_geometry(mesh));
    }

    return {};
}

template alp::Result<void, InvalidMesh> validate_impl<2, float>(const mesh::View_<2, float> &, ValidationFlags);
template alp::Result<void, InvalidMesh> validate_impl<2, double>(const mesh::View_<2, double> &, ValidationFlags);
template alp::Result<void, InvalidMesh> validate_impl<3, float>(const mesh::View_<3, float> &, ValidationFlags);
template alp::Result<void, InvalidMesh> validate_impl<3, double>(const mesh::View_<3, double> &, ValidationFlags);

}
}
