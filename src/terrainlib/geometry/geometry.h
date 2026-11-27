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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/component_wise.hpp>
#include <libassert/assert.hpp>
#include <radix/geometry.h>

#include "containers/FixedVector.h"
#include "glm_utils.h"
#include "std_compat.h"

namespace geometry {

// TODO: move to geometry/bounds.h
// Grow bounds by the same distance on every side.
template <glm::length_t n_dims, typename T>
radix::geometry::Aabb<n_dims, T> pad_bounds_absolute(const radix::geometry::Aabb<n_dims, T> &bounds, const T padding) {
    using Vec = glm::vec<n_dims, T>;
    return radix::geometry::Aabb<n_dims, T>(bounds.min - Vec(padding), bounds.max + Vec(padding));
}

// TODO: move to geometry/bounds.h
// Grow bounds by the given fraction of their size on every side.
template <glm::length_t n_dims, typename T>
radix::geometry::Aabb<n_dims, T> pad_bounds_relative(const radix::geometry::Aabb<n_dims, T> &bounds, const T fraction, const T min_padding = T(0)) {
    using Vec = glm::vec<n_dims, T>;
    const Vec padding = glm::max(bounds.size() * fraction, Vec(min_padding));
    return radix::geometry::Aabb<n_dims, T>(bounds.min - padding, bounds.max + padding);
}

// TODO: move to geometry/bounds.h
// Map `point` to [0, 1] within `bounds`, clamped.
template <glm::length_t n_dims, typename T>
glm::vec<n_dims, T> relativize_point(const radix::geometry::Aabb<n_dims, T> &bounds, const glm::vec<n_dims, T> &point) {
    using Vec = glm::vec<n_dims, T>;
    const Vec size = glm::max(bounds.size(), Vec(std::numeric_limits<T>::min()));
    return glm::clamp((point - bounds.min) / size, T(0), T(1));
}

// TODO: move to geometry/bounds.h
template <glm::length_t n_dims, typename T>
radix::geometry::Aabb<n_dims, T> union_bounds(const radix::geometry::Aabb<n_dims, T> &a, const radix::geometry::Aabb<n_dims, T> &b) {
    radix::geometry::Aabb<n_dims, T> bounds = a;
    bounds.expand_by(b);
    return bounds;
}

namespace detail {

// TODO: move to geometry/bounds.h
// Shifts `bounds` along `dimension` by whole periods so that it starts within [centre - period / 2, centre + period / 2), and splits it in two if it then reaches past that range's upper end.
template <glm::length_t n_dims, typename T>
FixedVector<radix::geometry::Aabb<n_dims, T>, 2> wrap_into_period(const radix::geometry::Aabb<n_dims, T> &bounds, const glm::length_t dimension, const T centre, const T period) {
    ASSERT(std::isfinite(bounds.min[dimension]) && std::isfinite(bounds.max[dimension]) && std::isfinite(centre));
    ASSERT(period > 0);
    const T lower_end = centre - period / 2;
    const T upper_end = centre + period / 2;
    T lower = bounds.min[dimension];
    T upper = bounds.max[dimension];
    if (upper < lower) {
        upper += period;
    }

    radix::geometry::Aabb<n_dims, T> first = bounds;
    if (upper - lower >= period) {
        first.min[dimension] = lower_end;
        first.max[dimension] = upper_end;
        return {first};
    }
    const T shift = period * std::floor((lower - lower_end) / period);
    lower -= shift;
    upper -= shift;
    first.min[dimension] = lower;
    if (upper > upper_end) {
        first.max[dimension] = upper_end;
        radix::geometry::Aabb<n_dims, T> second = bounds;
        second.min[dimension] = lower_end;
        second.max[dimension] = upper - period;
        return {first, second};
    }
    first.max[dimension] = upper;
    return {first};
}

} // namespace detail

// TODO: move to geometry/bounds.h
// Shifts `bounds` along each periodic dimension by whole periods so that it starts within [centre - period / 2, centre + period / 2), and splits it in two
// along each such dimension where it then reaches past that range's upper end. The input may lie in any period; max < min along a periodic dimension means the
// bounds run across the end of a period. Bounds spanning at least a period along a dimension give its whole range. Along periodic dimensions, the bounds and
// `centre` must be finite and `period` positive; along the others, `centre` and `period` are ignored.
template <glm::length_t n_dims, typename T>
std::vector<radix::geometry::Aabb<n_dims, T>> wrap_into_periods(
    const radix::geometry::Aabb<n_dims, T> &bounds, const glm::vec<n_dims, bool> &mask, const glm::vec<n_dims, T> &centre, const glm::vec<n_dims, T> &period) {
    std::vector<radix::geometry::Aabb<n_dims, T>> pieces{bounds};
    for (glm::length_t dimension = 0; dimension < n_dims; dimension++) {
        if (!mask[dimension]) {
            continue;
        }

        std::vector<radix::geometry::Aabb<n_dims, T>> wrapped;
        for (const radix::geometry::Aabb<n_dims, T> &piece : pieces) {
            const FixedVector<radix::geometry::Aabb<n_dims, T>, 2> split = detail::wrap_into_period(piece, dimension, centre[dimension], period[dimension]);
            std_compat::append_range(wrapped, split);
        }
        pieces = std::move(wrapped);
    }
    return pieces;
}

// TODO: move to geometry/bounds.h
// Whether the bounds enclose nothing, i.e., min >= max on some axis.
template <glm::length_t n_dims, typename T>
bool is_empty(const radix::geometry::Aabb<n_dims, T> &bounds) {
    return glm::any(glm::greaterThanEqual(bounds.min, bounds.max));
}

// TODO: move to geometry/bounds.h
// Whether every coordinate of the bounds is finite.
template <glm::length_t n_dims, typename T>
bool is_finite(const radix::geometry::Aabb<n_dims, T> &bounds) {
    return ::is_finite(bounds.min) && ::is_finite(bounds.max);
}

// TODO: move to geometry/bounds.h
template <glm::length_t n_dims, typename T>
T compute_surface_area(const radix::geometry::Aabb<n_dims, T> &bounds) {
    const glm::vec<n_dims, T> size = bounds.size();
    return glm::compAdd(size) * glm::compAdd(size) - glm::compAdd(size * size);
}

// TODO: move to geometry/bounds.h
// The mean of the extents, as a single measure of how large the bounds are.
template <glm::length_t n_dims, typename T>
T compute_mean_extent(const radix::geometry::Aabb<n_dims, T> &bounds) {
    return glm::compAdd(bounds.size()) / static_cast<T>(n_dims);
}

// TODO: move to geometry/bounds.h
// Gets the corner of `bounds` that reaches farthest in `direction`.
// One of them if several reach equally far.
template <glm::length_t n_dims, typename T>
glm::vec<n_dims, T> get_corner_in_direction(const radix::geometry::Aabb<n_dims, T> &bounds, const glm::vec<n_dims, T> &direction) {
    return glm::mix(bounds.min, bounds.max, glm::greaterThanEqual(direction, glm::vec<n_dims, T>(0)));
}

// TODO: move to geometry/bounds.h
// Finds the point of `bounds` farthest from `point`.
// One of them if several are equally far.
template <glm::length_t n_dims, typename T>
glm::vec<n_dims, T> find_farthest_point_in_bounds(const radix::geometry::Aabb<n_dims, T> &bounds, const glm::vec<n_dims, T> &point) {
    // The farthest point is the corner across the centre, and the offsets to both extreme corners add up to twice the offset to the centre.
    const glm::vec<n_dims, T> to_centre = (bounds.min - point) + (bounds.max - point);
    return get_corner_in_direction(bounds, to_centre);
}

// TODO: move to geometry/bounds.h
// Computes the distance from `point` to the farthest point of `bounds`.
template <glm::length_t n_dims, typename T>
T compute_farthest_distance_to_bounds(const radix::geometry::Aabb<n_dims, T> &bounds, const glm::vec<n_dims, T> &point) {
    return glm::distance(point, find_farthest_point_in_bounds(bounds, point));
}

// Normalize a set of positions into the range of [-1,1] based on maximum extents of the bounding box.
// Outputs are written as float coordinates.
// Optionally outputs the computed AABB if out_bounds is provided.
template <glm::length_t n_dims>
void to_approximate_normalized(
    std::span<const glm::vec<n_dims, double>> positions,
    std::vector<glm::vec<n_dims, float>> &approx,
    radix::geometry::Aabb<n_dims, double> *out_bounds = nullptr) {
    // compute bounds
    const radix::geometry::Aabb<n_dims, double> bounds = radix::geometry::find_bounds(positions);
    const glm::vec<n_dims, double> center = bounds.centre();
    const glm::vec<n_dims, double> extents = bounds.size() / 2.0;
    const double max_extents = glm::compMax(extents);

    if (out_bounds) {
        *out_bounds = bounds;
    }

    // normalize based on aabb
    approx.clear();
    approx.reserve(positions.size());
    for (const auto &p : positions) {
        const glm::vec<n_dims, double> rel = (p - center) / max_extents;
        approx.push_back(glm::vec<n_dims, float>(rel));
    }
}
template <glm::length_t n_dims>
void to_approximate_normalized(
    const std::vector<glm::vec<n_dims, double>> &positions,
    std::vector<glm::vec<n_dims, float>> &approx,
    radix::geometry::Aabb<n_dims, double> *out_bounds = nullptr) {
    return to_approximate_normalized(std::span(positions), approx, out_bounds);
}

// Normalize a set of positions into the range of [-1,1] based on maximum extents of the bounding box.
// Outputs are written as float coordinates.
// Optionally outputs the computed AABB if out_bounds is provided.
template <glm::length_t n_dims>
std::vector<glm::vec<n_dims, float>> to_approximate_normalized(
    std::span<const glm::vec<n_dims, double>> positions,
    radix::geometry::Aabb<n_dims, double> *out_bounds = nullptr) {
    std::vector<glm::vec<n_dims, float>> approx;
    to_approximate_normalized(positions, approx, out_bounds);
    return approx;
}
template <glm::length_t n_dims>
std::vector<glm::vec<n_dims, float>> to_approximate_normalized(
    const std::vector<glm::vec<n_dims, double>> &positions,
    radix::geometry::Aabb<n_dims, double> *out_bounds = nullptr) {
    return to_approximate_normalized(std::span(positions), out_bounds);
}

template <glm::length_t n_dims, typename T>
radix::geometry::Triangle<n_dims, T> corners(const glm::uvec3 &triangle, const std::span<const glm::vec<n_dims, T>> positions) {
    return {positions[triangle.x], positions[triangle.y], positions[triangle.z]};
}

// Scalar z-component in 2D, full cross product vector in 3D.
template <glm::length_t n_dims, typename T>
auto cross(const glm::vec<n_dims, T> &a, const glm::vec<n_dims, T> &b) {
    if constexpr (n_dims == 2) {
        return a.x * b.y - a.y * b.x;
    } else if constexpr (n_dims == 3) {
        return glm::cross(a, b);
    } else {
        static_assert(n_dims == 2 || n_dims == 3, "cross is only defined for 2D and 3D vectors");
    }
}

// Positive when p lies left of the directed edge a to b, and zero when the three are collinear.
template <typename T>
T orient(const glm::vec<2, T> &a, const glm::vec<2, T> &b, const glm::vec<2, T> &p) {
    return cross(b - a, p - a);
}

// Corner weights of the point, summing to one. Meaningless for a degenerate triangle.
template <glm::length_t n_dims, typename T>
glm::vec<3, T> compute_barycentric(
    const glm::vec<n_dims, T> &point,
    const glm::vec<n_dims, T> &a,
    const glm::vec<n_dims, T> &b,
    const glm::vec<n_dims, T> &c) {
    if constexpr (n_dims == 2) {
        const T denom = orient(a, b, c);
        const T first = orient(b, c, point) / denom;
        const T second = orient(c, a, point) / denom;
        return {first, second, T(1) - first - second};
    } else {
        using Vec = glm::vec<n_dims, T>;

        const Vec v0 = b - a;
        const Vec v1 = c - a;
        const Vec v2 = point - a;

        const T d00 = glm::dot(v0, v0);
        const T d01 = glm::dot(v0, v1);
        const T d11 = glm::dot(v1, v1);
        const T d20 = glm::dot(v2, v0);
        const T d21 = glm::dot(v2, v1);

        const T denom = d00 * d11 - d01 * d01;
        ASSERT(denom != 0);
        const T inv_denom = 1 / denom;

        const T v = (d11 * d20 - d01 * d21) * inv_denom;
        const T w = (d00 * d21 - d01 * d20) * inv_denom;
        const T u = 1 - v - w;

        return Vec(u, v, w);
    }
}

template <glm::length_t n_dims, typename T>
glm::vec<3, T> compute_barycentric(const glm::vec<n_dims, T> &point, const radix::geometry::Triangle<n_dims, T> &triangle) {
    return compute_barycentric(point, triangle[0], triangle[1], triangle[2]);
}

template <typename V, typename T>
V interpolate(const std::array<V, 3> &values, const glm::vec<3, T> &weights) {
    return weights[0] * values[0] + weights[1] * values[1] + weights[2] * values[2];
}

template <glm::length_t n_dims, typename T>
T distance_to_segment(const glm::vec<n_dims, T> &point, const radix::geometry::Edge<n_dims, T> &segment) {
    constexpr T epsilon_squared = radix::geometry::epsilon<T> * radix::geometry::epsilon<T>;

    const glm::vec<n_dims, T> along = segment[1] - segment[0];
    const T length_squared = glm::dot(along, along);
    if (length_squared <= epsilon_squared) {
        return glm::distance(point, segment[0]);
    }
    const T t = std::clamp(glm::dot(point - segment[0], along) / length_squared, T(0), T(1));
    return glm::distance(point, segment[0] + t * along);
}

// Zero inside the triangle, growing outside it. Accepts either winding.
template <typename T>
T distance_to_triangle(const glm::vec<2, T> &point, const radix::geometry::Triangle<2, T> &triangle) {
    const T orientation = cross(triangle[1] - triangle[0], triangle[2] - triangle[0]);

    bool inside = orientation != T(0);
    for (uint8_t corner = 0; inside && corner < 3; corner++) {
        const glm::vec<2, T> edge = triangle[(corner + 1) % 3] - triangle[corner];
        inside = cross(edge, point - triangle[corner]) * orientation >= T(0);
    }
    if (inside) {
        return T(0);
    }

    T nearest = std::numeric_limits<T>::max();
    for (uint8_t corner = 0; corner < 3; corner++) {
        const radix::geometry::Edge<2, T> edge = {triangle[corner], triangle[(corner + 1) % 3]};
        nearest = std::min(nearest, distance_to_segment(point, edge));
    }
    return nearest;
}

namespace detail {
// True when an edge normal of the first triangle separates the two. Touching does not separate.
template <typename T>
bool separates(const radix::geometry::Triangle<2, T> &triangle, const radix::geometry::Triangle<2, T> &other) {
    constexpr T epsilon_squared = radix::geometry::epsilon<T> * radix::geometry::epsilon<T>;

    const auto extent_along = [](const glm::vec<2, T> &axis, const radix::geometry::Triangle<2, T> &corners) {
        T lowest = std::numeric_limits<T>::max();
        T highest = std::numeric_limits<T>::lowest();
        for (const glm::vec<2, T> &corner : corners) {
            const T value = glm::dot(axis, corner);
            lowest = std::min(lowest, value);
            highest = std::max(highest, value);
        }
        return glm::vec<2, T>(lowest, highest);
    };

    for (uint8_t corner = 0; corner < 3; corner++) {
        const glm::vec<2, T> edge = triangle[(corner + 1) % 3] - triangle[corner];
        const glm::vec<2, T> axis(-edge.y, edge.x);
        if (glm::dot(axis, axis) <= epsilon_squared) {
            continue;
        }

        const glm::vec<2, T> own = extent_along(axis, triangle);
        const glm::vec<2, T> rest = extent_along(axis, other);
        if (own.y < rest.x || rest.y < own.x) {
            return true;
        }
    }
    return false;
}
} // namespace detail

// Separating axis test over the six edge normals. Triangles that only touch count as overlapping.
template <typename T>
bool triangles_overlap(const radix::geometry::Triangle<2, T> &a, const radix::geometry::Triangle<2, T> &b) {
    return !detail::separates(a, b) && !detail::separates(b, a);
}

template <typename T>
constexpr T epsilon() {
    if constexpr (std::is_same_v<T, float>) {
        return 1e-6f;
    } else if constexpr (std::is_same_v<T, double>) {
        return 1e-12;
    } else {
        static_assert(sizeof(T) == 0, "Unsupported type for epsilon");
    }
}
template <typename T>
constexpr T epsilon_sq() {
    return epsilon<T>() * epsilon<T>();
}

template <typename T>
glm::vec<3, T> compute_normal(const glm::vec<3, T> &a,
                              const glm::vec<3, T> &b,
                              const glm::vec<3, T> &c,
                              const bool normalize = true);
template <typename T>
glm::vec<3, T> compute_normal(const glm::uvec3 &triangle,
                              const std::span<const glm::vec<3, T>> positions,
                              const bool normalize = true);

template <glm::length_t n_dims, typename T>
T compute_triangle_area(const glm::vec<n_dims, T> &v0, const glm::vec<n_dims, T> &v1, const glm::vec<n_dims, T> &v2);
template <glm::length_t n_dims, typename T>
T compute_triangle_area(const std::array<glm::vec<n_dims, T>, 3> &triangle);
template <glm::length_t n_dims, typename T>
T compute_triangle_area(const glm::uvec3 &triangle, const std::span<const glm::vec<n_dims, T>> positions);
template <glm::length_t n_dims, typename T>
T compute_triangle_area(const glm::uvec3 &triangle, const std::vector<glm::vec<n_dims, T>>& positions);

// Negative where the triangle winds the other way.
template <typename T>
T compute_signed_triangle_area(const glm::vec<2, T> &v0, const glm::vec<2, T> &v1, const glm::vec<2, T> &v2);
template <typename T>
T compute_signed_triangle_area(const std::array<glm::vec<2, T>, 3> &triangle);
template <typename T>
T compute_signed_triangle_area(const glm::uvec3 &triangle, const std::span<const glm::vec<2, T>> positions);
template <typename T>
T compute_signed_triangle_area(const glm::uvec3 &triangle, const std::vector<glm::vec<2, T>> &positions);

template <glm::length_t n_dims, typename T>
T compute_squared_triangle_area(const glm::vec<n_dims, T> &v0, const glm::vec<n_dims, T> &v1, const glm::vec<n_dims, T> &v2);
template <glm::length_t n_dims, typename T>
T compute_squared_triangle_area(const std::array<glm::vec<n_dims, T>, 3> &triangle);
template <glm::length_t n_dims, typename T>
T compute_squared_triangle_area(const glm::uvec3 &triangle, const std::span<const glm::vec<n_dims, T>> positions);
template <glm::length_t n_dims, typename T>
T compute_squared_triangle_area(const glm::uvec3 &triangle, const std::vector<glm::vec<n_dims, T>> &positions);

template <glm::length_t n_dims, typename T>
bool is_empty_triangle(const glm::vec<n_dims, T> &v0, const glm::vec<n_dims, T> &v1, const glm::vec<n_dims, T> &v2, const T min_area = 2 * epsilon<T>());
template <glm::length_t n_dims, typename T>
bool is_empty_triangle(const std::array<glm::vec<n_dims, T>, 3> &triangle, const T min_area = 2 * epsilon<T>());
template <glm::length_t n_dims, typename T>
bool is_empty_triangle(const glm::uvec3 &triangle, const std::span<const glm::vec<n_dims, T>> positions, const T min_area = 2 * epsilon<T>());
template <glm::length_t n_dims, typename T>
bool is_empty_triangle(const glm::uvec3 &triangle, const std::vector<glm::vec<n_dims, T>> &positions, const T min_area = 2 * epsilon<T>());

} // namespace geometry

#include "geometry.inl"
