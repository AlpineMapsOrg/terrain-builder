#pragma once

#include <glm/glm.hpp>

namespace view {

// What a caller specifies about a camera.
struct CameraOptions {
    // The point it looks from.
    glm::dvec3 eye;
    // The direction it looks in.
    glm::dvec3 forward;
    // The direction that is up on screen.
    glm::dvec3 up;
    // The vertical field of view in radians.
    double fov_y;
    // The screen width over the screen height.
    double aspect;
    // The distance to the near plane.
    double z_near;

    bool operator==(const CameraOptions &) const = default;
};

}
