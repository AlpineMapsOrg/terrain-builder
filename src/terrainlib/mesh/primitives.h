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

#pragma once

#include <glm/glm.hpp>
#include <radix/geometry.h>

#include "mesh/SimpleMesh.h"

namespace mesh {

// A closed box filling `bounds`, wound outward.
inline Simple make_box(const radix::geometry::Aabb<3, double> &bounds) {
    const glm::dvec3 &min = bounds.min;
    const glm::dvec3 &max = bounds.max;
    return Simple(
        {
            {0, 2, 1}, {0, 3, 2},
            {4, 5, 6}, {4, 6, 7},
            {0, 1, 5}, {0, 5, 4},
            {1, 2, 6}, {1, 6, 5},
            {2, 3, 7}, {2, 7, 6},
            {3, 0, 4}, {3, 4, 7},
        },
        {
            {min.x, min.y, min.z},
            {max.x, min.y, min.z},
            {max.x, max.y, min.z},
            {min.x, max.y, min.z},
            {min.x, min.y, max.z},
            {max.x, min.y, max.z},
            {max.x, max.y, max.z},
            {min.x, max.y, max.z},
        });
}

// A closed box of `size` around `centre`, wound outward.
inline Simple make_box(const glm::dvec3 &centre, const glm::dvec3 &size) {
    const glm::dvec3 half_size = size / 2.0;
    const radix::geometry::Aabb<3, double> bounds{centre - half_size, centre + half_size};
    return make_box(bounds);
}

// A closed cube with sides of `size` around `centre`, wound outward.
inline Simple make_cube(const glm::dvec3 &centre, const double size) {
    return make_box(centre, glm::dvec3(size));
}

// A rectangle filling `bounds` at z = 0, made of two triangles facing +z.
inline Simple make_rect(const radix::geometry::Aabb<2, double> &bounds) {
    const glm::dvec2 &min = bounds.min;
    const glm::dvec2 &max = bounds.max;
    return Simple(
        {{0, 1, 2}, {0, 2, 3}},
        {
            {min.x, min.y, 0.0},
            {max.x, min.y, 0.0},
            {max.x, max.y, 0.0},
            {min.x, max.y, 0.0},
        });
}

// A rectangle of `size` around `centre` at z = 0, made of two triangles facing +z.
inline Simple make_rect(const glm::dvec2 &centre, const glm::dvec2 &size) {
    const glm::dvec2 half_size = size / 2.0;
    const radix::geometry::Aabb<2, double> bounds{centre - half_size, centre + half_size};
    return make_rect(bounds);
}

// A square with sides of `size` around `centre` at z = 0, made of two triangles facing +z.
inline Simple make_square(const glm::dvec2 &centre, const double size) {
    return make_rect(centre, glm::dvec2(size));
}

} // namespace mesh
