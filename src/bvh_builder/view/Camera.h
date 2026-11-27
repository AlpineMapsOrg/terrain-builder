#pragma once

#include <string>
#include <utility>

#include <fmt/format.h>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#include "error/Result.h"
#include "error/utils.h"
#include "fmt_impls.h"
#include "geometry/geometry.h"

#include "view/CameraOptions.h"
#include "view/InvalidCamera.h"

namespace view {

// A perspective camera with a view.
class Camera {
public:
    // Creates the camera that `options` describe.
    // Fails with `InvalidCamera` if `forward` and `up` do not span a plane, `fov_y` is not between zero and pi, or `aspect` or `z_near` is not positive.
    [[nodiscard]]
    static alp::Result<Camera, InvalidCamera> create(const CameraOptions &options) {
        const auto reject = [&](std::string reason) {
            return alp::make_error(InvalidCamera{std::move(reason), options});
        };

        if (!(options.fov_y > 0.0 && options.fov_y < glm::pi<double>())) {
            return reject(fmt::format("the field of view of {} is not between zero and pi", options.fov_y));
        }
        if (!(options.aspect > 0.0)) {
            return reject(fmt::format("the aspect of {} is not positive", options.aspect));
        }
        if (!(options.z_near > 0.0)) {
            return reject(fmt::format("the near distance of {} is not positive", options.z_near));
        }
        if (!(glm::length(options.forward) > 0.0)) {
            return reject(fmt::format("forward {} has no length", options.forward));
        }
        if (!(glm::length(options.up) > 0.0)) {
            return reject(fmt::format("up {} has no length", options.up));
        }

        const glm::dvec3 forward = glm::normalize(options.forward);
        const glm::dvec3 sideways = glm::cross(forward, glm::normalize(options.up));
        if (!(glm::length(sideways) > geometry::epsilon<double>())) {
            return reject(fmt::format("up {} runs along forward {}", options.up, options.forward));
        }

        const glm::dvec3 right = glm::normalize(sideways);
        return Camera::create_unchecked({
            .eye = options.eye,
            .forward = forward,
            .up = glm::cross(right, forward),
            .fov_y = options.fov_y,
            .aspect = options.aspect,
            .z_near = options.z_near,
        });
    }

    // Creates the camera that `options` describe without checking them.
    // `forward` and `up` must be unit directions at right angles, `fov_y` between zero and pi, and `aspect` and `z_near` positive.
    [[nodiscard]]
    static Camera create_unchecked(const CameraOptions &options) {
        return Camera(options);
    }

    // The point it looks from.
    const glm::dvec3 &eye() const {
        return this->_options.eye;
    }
    // The unit direction it looks in.
    const glm::dvec3 &forward() const {
        return this->_options.forward;
    }
    // The unit direction that is right on screen.
    glm::dvec3 right() const {
        return glm::cross(this->_options.forward, this->_options.up);
    }
    // The unit direction that is up on screen, perpendicular to `forward`.
    const glm::dvec3 &up() const {
        return this->_options.up;
    }
    // The vertical field of view in radians.
    double fov_y() const {
        return this->_options.fov_y;
    }
    // The screen width over the screen height.
    double aspect() const {
        return this->_options.aspect;
    }
    // The distance to the near plane.
    double z_near() const {
        return this->_options.z_near;
    }

private:
    explicit Camera(const CameraOptions &options) : _options(options) {}

    // Its options, with `forward` and `up` as unit directions at right angles.
    CameraOptions _options;
};

}
