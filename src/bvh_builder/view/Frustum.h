#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <span>

#include <glm/glm.hpp>
#include <radix/geometry.h>

#include "geometry/CoordFrame.h"
#include "geometry/bounds.h"

#include "view/Camera.h"

namespace view {

namespace detail {

// Gets the corner among `corners` that reaches farthest in `direction`.
// One of them if several reach equally far.
inline glm::dvec3 get_corner_in_direction(const std::span<const glm::dvec3> corners, const glm::dvec3 &direction) {
    return std::ranges::max(corners, {}, [&](const glm::dvec3 &corner) {
        return glm::dot(corner, direction);
    });
}

// Computes how far `bounds` and the shape with `frustum_corners` are apart along `direction`.
// Negative if they overlap along it.
inline double compute_separation(const radix::geometry::Aabb3d &bounds, const std::span<const glm::dvec3> frustum_corners, const glm::dvec3 &direction) {
    const glm::dvec3 bounds_point = geometry::get_corner_in_direction(bounds, direction);
    const glm::dvec3 frustum_point = get_corner_in_direction(frustum_corners, -direction);
    return glm::dot(frustum_point - bounds_point, direction);
}

}

// The region a camera sees, without a far end.
class Frustum {
public:
    // Builds the frustum that `camera` sees.
    [[nodiscard]]
    static Frustum from_camera(const Camera &camera) {
        const CoordFrame frame{.origin = camera.eye(), .tangent = camera.right(), .bitangent = camera.up(), .normal = camera.forward()};

        const double half_height = camera.z_near() * std::tan(camera.fov_y() / 2.0);
        const double half_width = half_height * camera.aspect();

        Frustum frustum;
        frustum._eye = camera.eye();
        frustum._forward = camera.forward();
        frustum._z_near = camera.z_near();
        frustum._near_corners = {
            frame.to_world({-half_width, -half_height, camera.z_near()}),
            frame.to_world({half_width, -half_height, camera.z_near()}),
            frame.to_world({-half_width, half_height, camera.z_near()}),
            frame.to_world({half_width, half_height, camera.z_near()}),
        };
        std::ranges::transform(frustum._near_corners, frustum._side_edges.begin(), [&](const glm::dvec3 &corner) {
            return corner - camera.eye();
        });
        frustum._near_edges = {camera.right(), camera.up()};
        // Each side face spans the two side edges it lies between.
        frustum._face_normals = {
            glm::cross(frustum._side_edges[0], frustum._side_edges[1]),
            glm::cross(frustum._side_edges[2], frustum._side_edges[3]),
            glm::cross(frustum._side_edges[0], frustum._side_edges[2]),
            glm::cross(frustum._side_edges[1], frustum._side_edges[3]),
            camera.forward(),
        };
        return frustum;
    }

    friend bool intersects(const radix::geometry::Aabb3d &bounds, const Frustum &frustum);

private:
    Frustum() = default;

    // The corners of the part of the frustum up to `depth` in front of the eye: the near rectangle, then the rectangle at `depth`.
    std::array<glm::dvec3, 8> compute_corners(const double depth) const {
        // Nothing of the frustum lies closer than the near rectangle.
        const double scale = std::max(depth, this->_z_near) / this->_z_near;

        std::array<glm::dvec3, 8> corners;
        std::ranges::copy(this->_near_corners, corners.begin());
        std::ranges::transform(this->_side_edges, corners.begin() + 4, [&](const glm::dvec3 &edge) {
            return this->_eye + scale * edge;
        });
        return corners;
    }

    glm::dvec3 _eye;
    // The unit direction the camera looks in.
    glm::dvec3 _forward;
    double _z_near;
    // The corners of the near rectangle: bottom left, bottom right, top left, top right.
    std::array<glm::dvec3, 4> _near_corners;
    // The edges that run from the eye through each near corner, in the same order and reaching up to it.
    std::array<glm::dvec3, 4> _side_edges;
    // The directions of the edges of the near rectangle.
    std::array<glm::dvec3, 2> _near_edges;
    // The normals of the bottom, top, left, right and near faces.
    std::array<glm::dvec3, 5> _face_normals;
};

// Checks whether any point of `bounds` lies in `frustum`.
[[nodiscard]]
inline bool intersects(const radix::geometry::Aabb3d &bounds, const Frustum &frustum) {
    // Cutting the frustum off behind the bounds changes nothing about what the two share, and gives it a farthest corner in every direction.
    const double depth = glm::dot(frustum._forward, geometry::get_corner_in_direction(bounds, frustum._forward) - frustum._eye);
    const std::array<glm::dvec3, 8> frustum_corners = frustum.compute_corners(depth);

    // Two convex polyhedra share no point exactly when they are apart along a normal of one of their faces or along the cross product of an edge of each.
    const auto separates = [&](const glm::dvec3 &axis) {
        return detail::compute_separation(bounds, frustum_corners, axis) > 0.0 || detail::compute_separation(bounds, frustum_corners, -axis) > 0.0;
    };

    // The axes are both the face normals and the edge directions of the bounds.
    const std::array<glm::dvec3, 3> box_axes{glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(0.0, 0.0, 1.0)};
    const auto separates_across = [&](const glm::dvec3 &frustum_edge) {
        return std::ranges::any_of(box_axes, [&](const glm::dvec3 &box_axis) {
            return separates(glm::cross(box_axis, frustum_edge));
        });
    };

    const bool is_separated_along_normal = std::ranges::any_of(box_axes, separates) || std::ranges::any_of(frustum._face_normals, separates);
    const bool is_separated_along_edge_cross = std::ranges::any_of(frustum._side_edges, separates_across) || std::ranges::any_of(frustum._near_edges, separates_across);
    return !is_separated_along_normal && !is_separated_along_edge_cross;
}

}
