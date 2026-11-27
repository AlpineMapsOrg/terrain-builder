/*****************************************************************************
 * AlpineMaps.org
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

#include <algorithm>
#include <array>
#include <compare>
#include <ranges>
#include <vector>

#include <glm/common.hpp>

#include "geometry/compare.h"
#include "mesh/SimpleMesh.h"

template <glm::length_t n_dims, typename T>
TriangleSoup_<n_dims, T> to_triangle_soup(const mesh::Simple_<n_dims, T> &mesh) {
    TriangleSoup_<n_dims, T> soup;
    soup.reserve(mesh.triangles.size());
    for (const glm::uvec3 &triangle : mesh.triangles) {
        soup.push_back({mesh.positions[triangle[0]],
                        mesh.positions[triangle[1]],
                        mesh.positions[triangle[2]]});
    }
    return soup;
}

namespace detail {
template <glm::length_t n_dims, typename T>
void normalize_triangle(std::array<glm::vec<n_dims, T>, 3> &triangle) {
    auto min_it = std::ranges::min_element(triangle, {}, [](const auto &vertex) { return geometry::Strong{vertex}; });
    std::rotate(triangle.begin(), min_it, triangle.end());
}
} // namespace detail

template <glm::length_t n_dims, typename T>
void sort_triangle_soup(TriangleSoup_<n_dims, T> &soup) {
    // Rotate to put the smallest vertex first
    for (auto &triangle : soup) {
        detail::normalize_triangle(triangle);
    }

    // Sort the list of triangles
    std::ranges::sort(soup, {}, [](const auto &triangle) { return geometry::Strong{triangle}; });
}

template <glm::length_t n_dims, typename T>
TriangleSoup_<n_dims, T> to_sorted_triangle_soup(const mesh::Simple_<n_dims, T> &mesh) {
    TriangleSoup_<n_dims, T> soup = to_triangle_soup(mesh);
    sort_triangle_soup(soup);
    return soup;
}

template <glm::length_t n_dims, typename T>
auto to_sorted_triangle_soups(const std::vector<mesh::Simple_<n_dims, T>> &meshes) {
    std::vector<TriangleSoup_<n_dims, T>> soups;
    soups.reserve(meshes.size());
    for (const auto &mesh : meshes) {
        soups.push_back(to_sorted_triangle_soup(mesh));
    }
    std::ranges::sort(soups, [](const auto &a, const auto &b) { return geometry::compare_strong(a, b) == std::strong_ordering::less; });
    return soups;
}
