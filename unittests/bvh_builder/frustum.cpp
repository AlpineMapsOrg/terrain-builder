#include "../catch2_helpers.h"
#include "../result_helpers.h"

#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <radix/geometry.h>

#include "view/Camera.h"
#include "view/Frustum.h"

TEST_CASE("intersects keeps bounds that reach into the frustum", "[view][Frustum]") {
    // Sees x >= 11, |z - 30| <= x - 10 and |y - 20| <= 2 * (x - 10).
    const view::Camera camera = REQUIRE_VALUE(view::Camera::create({.eye = {10.0, 20.0, 30.0}, .forward = {1.0, 0.0, 0.0}, .up = {0.0, 0.0, 1.0}, .fov_y = glm::half_pi<double>(), .aspect = 2.0, .z_near = 1.0}));
    const view::Frustum frustum = view::Frustum::from_camera(camera);

    SECTION("entirely in view") {
        CHECK(view::intersects(radix::geometry::Aabb3d{{14.0, 19.0, 29.0}, {16.0, 21.0, 31.0}}, frustum));
    }
    SECTION("across the near plane") {
        CHECK(view::intersects(radix::geometry::Aabb3d{{10.5, 19.9, 29.9}, {11.5, 20.1, 30.1}}, frustum));
    }
    SECTION("touching the near plane") {
        CHECK(view::intersects(radix::geometry::Aabb3d{{10.5, 19.9, 29.9}, {11.0, 20.1, 30.1}}, frustum));
    }
    SECTION("across the top") {
        CHECK(view::intersects(radix::geometry::Aabb3d{{14.0, 19.0, 34.0}, {16.0, 21.0, 37.0}}, frustum));
    }
    SECTION("across the bottom") {
        CHECK(view::intersects(radix::geometry::Aabb3d{{14.0, 19.0, 23.0}, {16.0, 21.0, 26.0}}, frustum));
    }
    SECTION("across the left") {
        CHECK(view::intersects(radix::geometry::Aabb3d{{14.0, 31.0, 29.0}, {16.0, 34.0, 31.0}}, frustum));
    }
    SECTION("across the right") {
        CHECK(view::intersects(radix::geometry::Aabb3d{{14.0, 6.0, 29.0}, {16.0, 9.0, 31.0}}, frustum));
    }
    SECTION("around the eye") {
        CHECK(view::intersects(radix::geometry::Aabb3d{{0.0, 10.0, 20.0}, {20.0, 30.0, 40.0}}, frustum));
    }
}

TEST_CASE("intersects rejects bounds outside the frustum", "[view][Frustum]") {
    // Sees x >= 11, |z - 30| <= x - 10 and |y - 20| <= 2 * (x - 10).
    const view::Camera camera = REQUIRE_VALUE(view::Camera::create({.eye = {10.0, 20.0, 30.0}, .forward = {1.0, 0.0, 0.0}, .up = {0.0, 0.0, 1.0}, .fov_y = glm::half_pi<double>(), .aspect = 2.0, .z_near = 1.0}));
    const view::Frustum frustum = view::Frustum::from_camera(camera);

    SECTION("behind the eye") {
        CHECK_FALSE(view::intersects(radix::geometry::Aabb3d{{4.0, 19.0, 29.0}, {6.0, 21.0, 31.0}}, frustum));
    }
    SECTION("closer than the near plane") {
        CHECK_FALSE(view::intersects(radix::geometry::Aabb3d{{10.2, 19.9, 29.9}, {10.8, 20.1, 30.1}}, frustum));
    }
    SECTION("above") {
        CHECK_FALSE(view::intersects(radix::geometry::Aabb3d{{14.0, 19.0, 37.0}, {16.0, 21.0, 38.0}}, frustum));
    }
    SECTION("below") {
        CHECK_FALSE(view::intersects(radix::geometry::Aabb3d{{14.0, 19.0, 22.0}, {16.0, 21.0, 23.0}}, frustum));
    }
    SECTION("to the left") {
        CHECK_FALSE(view::intersects(radix::geometry::Aabb3d{{14.0, 33.0, 29.0}, {16.0, 34.0, 31.0}}, frustum));
    }
    SECTION("to the right") {
        CHECK_FALSE(view::intersects(radix::geometry::Aabb3d{{14.0, 6.0, 29.0}, {16.0, 7.0, 31.0}}, frustum));
    }
}

TEST_CASE("intersects keeps bounds that just reach into the frustum of a tilted camera", "[view][Frustum]") {
    // A point is in view if x + 2y + 2z >= 9, |2x + y - 2z| <= (x + 2y + 2z) / 2 and |-2x + 2y - z| <= (x + 2y + 2z) / 2.
    const view::Camera camera = REQUIRE_VALUE(view::Camera::create({.eye = {0.0, 0.0, 0.0}, .forward = {1.0, 2.0, 2.0}, .up = {2.0, 1.0, -2.0}, .fov_y = 2.0 * std::atan(0.5), .aspect = 1.0, .z_near = 3.0}));
    const view::Frustum frustum = view::Frustum::from_camera(camera);

    SECTION("across its right face") {
        CHECK(view::intersects(radix::geometry::Aabb3d{{-0.2, 5.8, 2.8}, {0.2, 6.2, 3.2}}, frustum));
    }
    SECTION("over its lowest corner") {
        CHECK(view::intersects(radix::geometry::Aabb3d{{-9.0, -1.0, -6.5}, {11.0, 0.6, 13.5}}, frustum));
    }
    SECTION("passing inside a side edge") {
        CHECK(view::intersects(radix::geometry::Aabb3d{{0.0, 6.55, 0.98}, {4.0, 6.65, 1.02}}, frustum));
    }
    SECTION("passing inside an edge of the near rectangle") {
        CHECK(view::intersects(radix::geometry::Aabb3d{{0.0, 2.6, 0.95}, {4.0, 2.7, 1.05}}, frustum));
    }
}

TEST_CASE("intersects rejects bounds that just miss the frustum of a tilted camera", "[view][Frustum]") {
    // A point is in view if x + 2y + 2z >= 9, |2x + y - 2z| <= (x + 2y + 2z) / 2 and |-2x + 2y - z| <= (x + 2y + 2z) / 2.
    const view::Camera camera = REQUIRE_VALUE(view::Camera::create({.eye = {0.0, 0.0, 0.0}, .forward = {1.0, 2.0, 2.0}, .up = {2.0, 1.0, -2.0}, .fov_y = 2.0 * std::atan(0.5), .aspect = 1.0, .z_near = 3.0}));
    const view::Frustum frustum = view::Frustum::from_camera(camera);

    SECTION("past its right face") {
        // Every point in view has 5x - 2y + 4z >= 0, which is at most -0.5 here.
        CHECK_FALSE(view::intersects(radix::geometry::Aabb3d{{-0.5, 6.0, 2.6}, {-0.1, 6.4, 3.0}}, frustum));
    }
    SECTION("under its lowest corner") {
        // Every point in view has y >= 0.5.
        CHECK_FALSE(view::intersects(radix::geometry::Aabb3d{{-9.0, -1.0, -6.5}, {11.0, 0.4, 13.5}}, frustum));
    }
    SECTION("passing outside a side edge") {
        // Every point in view has y <= 7z, and here y is at least 7.35 while 7z is at most 7.14.
        CHECK_FALSE(view::intersects(radix::geometry::Aabb3d{{0.0, 7.35, 0.98}, {4.0, 7.45, 1.02}}, frustum));
    }
    SECTION("passing outside an edge of the near rectangle") {
        // Every point in view has y + 2z >= 4.5, which is at most 4.45 here.
        CHECK_FALSE(view::intersects(radix::geometry::Aabb3d{{0.0, 2.25, 0.95}, {4.0, 2.35, 1.05}}, frustum));
    }
}
