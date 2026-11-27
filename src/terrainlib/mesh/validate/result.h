/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2025 Martin Braunsperger
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

#include <glm/glm.hpp>

#include "error/MessageCode.h"
#include "error/Result.h"
#include "mesh/SimpleMesh.h"
#include "mesh/View.h"
#include "mesh/validate/flags.h"

namespace mesh {

struct InvalidMesh : alp::MessageCode<"the mesh is invalid"> {};

template <glm::length_t n_dims, typename T>
[[nodiscard]]
alp::Result<void, InvalidMesh> validate(const mesh::Simple_<n_dims, T> &mesh, ValidationFlags flags = default_validation_flags);
template <glm::length_t n_dims, typename T>
[[nodiscard]]
alp::Result<void, InvalidMesh> validate(const mesh::View_<n_dims, T> &mesh, ValidationFlags flags = default_validation_flags);

template <glm::length_t n_dims, typename T>
[[nodiscard]]
alp::Result<void, InvalidMesh> validate_basic(const mesh::Simple_<n_dims, T> &mesh);
template <glm::length_t n_dims, typename T>
[[nodiscard]]
alp::Result<void, InvalidMesh> validate_basic(const mesh::View_<n_dims, T> &mesh);

template <glm::length_t n_dims, typename T>
[[nodiscard]]
alp::Result<void, InvalidMesh> validate_unconnected(const mesh::Simple_<n_dims, T> &mesh);
template <glm::length_t n_dims, typename T>
[[nodiscard]]
alp::Result<void, InvalidMesh> validate_unconnected(const mesh::View_<n_dims, T> &mesh);

template <glm::length_t n_dims, typename T>
[[nodiscard]]
alp::Result<void, InvalidMesh> validate_connected(const mesh::View_<n_dims, T> &mesh);
template <glm::length_t n_dims, typename T>
[[nodiscard]]
alp::Result<void, InvalidMesh> validate_connected(const mesh::Simple_<n_dims, T> &mesh);

template <glm::length_t n_dims, typename T>
[[nodiscard]]
alp::Result<void, InvalidMesh> validate_manifold(const mesh::View_<n_dims, T> &mesh);
template <glm::length_t n_dims, typename T>
[[nodiscard]]
alp::Result<void, InvalidMesh> validate_manifold(const mesh::Simple_<n_dims, T> &mesh);

} // namespace mesh

#include "mesh/validate/result.inl"
