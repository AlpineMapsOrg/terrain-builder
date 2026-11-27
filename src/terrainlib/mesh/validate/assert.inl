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

#include <source_location>

#include "error/unwrap.h"
#include "mesh/validate/assert.h"
#include "mesh/validate/result.h"

namespace mesh {

template <glm::length_t n_dims, typename T>
void assert_valid(const mesh::View_<n_dims, T> &mesh, const ValidationFlags flags, const std::source_location location) {
    alp::unwrap(validate(mesh, flags), location);
}
template <glm::length_t n_dims, typename T>
void assert_valid(const mesh::Simple_<n_dims, T> &mesh, const ValidationFlags flags, const std::source_location location) {
    assert_valid(mesh::View_<n_dims, T>(mesh), flags, location);
}

template <glm::length_t n_dims, typename T>
void assert_valid_basic(const mesh::View_<n_dims, T> &mesh, const std::source_location location) {
    alp::unwrap(validate_basic(mesh), location);
}
template <glm::length_t n_dims, typename T>
void assert_valid_basic(const mesh::Simple_<n_dims, T> &mesh, const std::source_location location) {
    assert_valid_basic(mesh::View_<n_dims, T>(mesh), location);
}

template <glm::length_t n_dims, typename T>
void assert_valid_unconnected(const mesh::View_<n_dims, T> &mesh, const std::source_location location) {
    alp::unwrap(validate_unconnected(mesh), location);
}
template <glm::length_t n_dims, typename T>
void assert_valid_unconnected(const mesh::Simple_<n_dims, T> &mesh, const std::source_location location) {
    assert_valid_unconnected(mesh::View_<n_dims, T>(mesh), location);
}

template <glm::length_t n_dims, typename T>
void assert_valid_connected(const mesh::View_<n_dims, T> &mesh, const std::source_location location) {
    alp::unwrap(validate_connected(mesh), location);
}
template <glm::length_t n_dims, typename T>
void assert_valid_connected(const mesh::Simple_<n_dims, T> &mesh, const std::source_location location) {
    assert_valid_connected(mesh::View_<n_dims, T>(mesh), location);
}

template <glm::length_t n_dims, typename T>
void assert_valid_manifold(const mesh::View_<n_dims, T> &mesh, const std::source_location location) {
    alp::unwrap(validate_manifold(mesh), location);
}
template <glm::length_t n_dims, typename T>
void assert_valid_manifold(const mesh::Simple_<n_dims, T> &mesh, const std::source_location location) {
    assert_valid_manifold(mesh::View_<n_dims, T>(mesh), location);
}

} // namespace mesh
