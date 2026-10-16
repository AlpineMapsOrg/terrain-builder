#pragma once

#include <limits>

#include <glm/glm.hpp>
#include <glm/gtx/component_wise.hpp>
#include <radix/geometry.h>

#include "range/RangeInclusive.h"

namespace geometry {

// Grow bounds by the same distance on every side.
template <glm::length_t n_dims, typename T>
radix::geometry::Aabb<n_dims, T> pad_bounds_absolute(const radix::geometry::Aabb<n_dims, T> &bounds, const T padding) {
    using Vec = glm::vec<n_dims, T>;
    return radix::geometry::Aabb<n_dims, T>(bounds.min - Vec(padding), bounds.max + Vec(padding));
}

// Grow bounds by the given fraction of their size on every side.
template <glm::length_t n_dims, typename T>
radix::geometry::Aabb<n_dims, T> pad_bounds_relative(const radix::geometry::Aabb<n_dims, T> &bounds, const T fraction, const T min_padding = T(0)) {
    using Vec = glm::vec<n_dims, T>;
    const Vec padding = glm::max(bounds.size() * fraction, Vec(min_padding));
    return radix::geometry::Aabb<n_dims, T>(bounds.min - padding, bounds.max + padding);
}

// Map `point` to [0, 1] within `bounds`, clamped.
template <glm::length_t n_dims, typename T>
glm::vec<n_dims, T> relativize_point(const radix::geometry::Aabb<n_dims, T> &bounds, const glm::vec<n_dims, T> &point) {
    using Vec = glm::vec<n_dims, T>;
    const Vec size = glm::max(bounds.size(), Vec(std::numeric_limits<T>::min()));
    return glm::clamp((point - bounds.min) / size, T(0), T(1));
}

template <glm::length_t n_dims, typename T>
radix::geometry::Aabb<n_dims, T> union_bounds(const radix::geometry::Aabb<n_dims, T> &a, const radix::geometry::Aabb<n_dims, T> &b) {
    radix::geometry::Aabb<n_dims, T> bounds = a;
    bounds.expand_by(b);
    return bounds;
}

template <glm::length_t n_dims, typename T>
T compute_surface_area(const radix::geometry::Aabb<n_dims, T> &bounds) {
    const glm::vec<n_dims, T> size = bounds.size();
    return glm::compAdd(size) * glm::compAdd(size) - glm::compAdd(size * size);
}

// The mean of the extents, as a single measure of how large the bounds are.
template <glm::length_t n_dims, typename T>
T compute_mean_extent(const radix::geometry::Aabb<n_dims, T> &bounds) {
    return glm::compAdd(bounds.size()) / static_cast<T>(n_dims);
}

// Gets the corner of `bounds` that reaches farthest in `direction`.
// One of them if several reach equally far.
template <glm::length_t n_dims, typename T>
glm::vec<n_dims, T> get_corner_in_direction(const radix::geometry::Aabb<n_dims, T> &bounds, const glm::vec<n_dims, T> &direction) {
    return glm::mix(bounds.min, bounds.max, glm::greaterThanEqual(direction, glm::vec<n_dims, T>(0)));
}

// Finds the point of `bounds` nearest to `point`, which is `point` itself if it lies inside them.
template <glm::length_t n_dims, typename T>
glm::vec<n_dims, T> find_nearest_point_in_bounds(const radix::geometry::Aabb<n_dims, T> &bounds, const glm::vec<n_dims, T> &point) {
    return glm::clamp(point, bounds.min, bounds.max);
}

// Finds the point of `bounds` farthest from `point`.
// One of them if several are equally far.
template <glm::length_t n_dims, typename T>
glm::vec<n_dims, T> find_farthest_point_in_bounds(const radix::geometry::Aabb<n_dims, T> &bounds, const glm::vec<n_dims, T> &point) {
    // The farthest point is the corner across the centre, and the offsets to both extreme corners add up to twice the offset to the centre.
    const glm::vec<n_dims, T> to_centre = (bounds.min - point) + (bounds.max - point);
    return get_corner_in_direction(bounds, to_centre);
}

// Computes the distance from `point` to the nearest point of `bounds`.
// Zero if `point` lies inside them.
template <glm::length_t n_dims, typename T>
T compute_nearest_distance_to_bounds(const radix::geometry::Aabb<n_dims, T> &bounds, const glm::vec<n_dims, T> &point) {
    return glm::distance(point, find_nearest_point_in_bounds(bounds, point));
}

// Computes the distance from `point` to the farthest point of `bounds`.
template <glm::length_t n_dims, typename T>
T compute_farthest_distance_to_bounds(const radix::geometry::Aabb<n_dims, T> &bounds, const glm::vec<n_dims, T> &point) {
    return glm::distance(point, find_farthest_point_in_bounds(bounds, point));
}

// Computes the distances from `point` to the nearest and the farthest point of `bounds`.
template <glm::length_t n_dims, typename T>
RangeInclusive<T> compute_distance_range_to_bounds(const radix::geometry::Aabb<n_dims, T> &bounds, const glm::vec<n_dims, T> &point) {
    return RangeInclusive<T>(
        compute_nearest_distance_to_bounds(bounds, point), compute_farthest_distance_to_bounds(bounds, point)
    );
}

// How bounds lie relative to a plane.
enum class PlaneRelation {
    // Entirely on the side the normal points to.
    Front,
    // Entirely on the other side.
    Back,
    // Reaching both sides, or touching the plane.
    Intersecting,
};

// Relates `bounds` to `plane`.
template <typename T>
PlaneRelation relate_to_plane(const radix::geometry::Aabb<3, T> &bounds, const radix::geometry::Plane<T> &plane) {
    const T front_corner_distance = radix::geometry::distance(plane, get_corner_in_direction(bounds, plane.normal));
    if (front_corner_distance < 0) {
        return PlaneRelation::Back;
    }

    const T back_corner_distance = radix::geometry::distance(plane, get_corner_in_direction(bounds, -plane.normal));
    if (back_corner_distance > 0) {
        return PlaneRelation::Front;
    }

    return PlaneRelation::Intersecting;
}

} // namespace geometry
