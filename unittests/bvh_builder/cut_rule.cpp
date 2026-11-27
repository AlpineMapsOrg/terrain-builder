#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "node_header_helpers.h"

#include <cmath>

#include <glm/glm.hpp>

#include "Range.h"

#include "bvh/NodeHeader.h"
#include "view/Camera.h"
#include "view/CutRule.h"

TEST_CASE("crosses_cut tells whether the cut can pass between the errors of a header", "[view][CutRule]") {
    // An error at a distance shows as their ratio, so it fits if it is at most half the distance.
    const view::Camera camera = REQUIRE_VALUE(view::Camera::create({.eye = {0.0, 0.0, 0.0}, .forward = {1.0, 0.0, 0.0}, .up = {0.0, 0.0, 1.0}, .fov_y = 2.0 * std::atan(0.5), .aspect = 1.0, .z_near = 1.0}));
    const view::CutRule rule{.threshold = 0.5};

    SECTION("the error fits and the parent's does not") {
        CHECK(view::crosses_cut(make_node_header({4.0, 6.0}, {}), RangeInclusive<double>(10.0), camera, rule));
    }
    SECTION("the error does not fit") {
        CHECK_FALSE(view::crosses_cut(make_node_header({6.0, 8.0}, {}), RangeInclusive<double>(10.0), camera, rule));
    }
    SECTION("the parent's error fits as well") {
        CHECK_FALSE(view::crosses_cut(make_node_header({2.0, 4.0}, {}), RangeInclusive<double>(10.0), camera, rule));
    }
    SECTION("the smallest error fits only from the farthest distance") {
        CHECK(view::crosses_cut(make_node_header({9.0, 12.0}, {}), RangeInclusive<double>(10.0, 20.0), camera, rule));
    }
    SECTION("the largest parent error fails to fit only from the nearest distance") {
        CHECK(view::crosses_cut(make_node_header({2.0, 7.0}, {}), RangeInclusive<double>(10.0, 20.0), camera, rule));
    }
    SECTION("closer than the near distance, where errors are measured at it") {
        CHECK(view::crosses_cut(make_node_header({0.4, 0.6}, {}), RangeInclusive<double>(0.0), camera, rule));
    }
}

TEST_CASE("is_small_enough_to_draw measures from the nearest point of the bounds", "[view][CutRule]") {
    // An error at a distance shows as their ratio, so it fits if it is at most half the distance.
    const view::Camera camera = REQUIRE_VALUE(view::Camera::create({.eye = {0.0, 0.0, 0.0}, .forward = {1.0, 0.0, 0.0}, .up = {0.0, 0.0, 1.0}, .fov_y = 2.0 * std::atan(0.5), .aspect = 1.0, .z_near = 1.0}));
    const view::CutRule rule{.threshold = 0.5};
    // The nearest point lies at a distance of 10 and the farthest at 12.
    const bvh::Bounds bounds{{10.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};

    SECTION("the error fits at the nearest point") {
        CHECK(view::is_small_enough_to_draw(4.5, bounds, camera, rule));
    }
    SECTION("the error fits only at the farthest point") {
        CHECK_FALSE(view::is_small_enough_to_draw(5.5, bounds, camera, rule));
    }
}

TEST_CASE("should_visit_leaf measures from the LOD bounds", "[view][CutRule]") {
    // An error at a distance shows as their ratio, so it fits if it is at most half the distance.
    const view::Camera camera = REQUIRE_VALUE(view::Camera::create({.eye = {0.0, 0.0, 0.0}, .forward = {1.0, 0.0, 0.0}, .up = {0.0, 0.0, 1.0}, .fov_y = 2.0 * std::atan(0.5), .aspect = 1.0, .z_near = 1.0}));
    const view::CutRule rule{.threshold = 0.5};
    // The nearest point lies at a distance of 10 and the farthest at 12.
    const bvh::Bounds lod_bounds{{10.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
    // So far away that every error here fits.
    const bvh::Bounds tight_bounds{{100.0, 0.0, 0.0}, {102.0, 0.0, 0.0}};

    SECTION("the error fits at the farthest point and the parent's does not at the nearest") {
        CHECK(view::should_visit_leaf(make_node_header({5.5, 5.5}, lod_bounds, tight_bounds), camera, rule));
    }
    SECTION("the parent's error fits as well") {
        CHECK_FALSE(view::should_visit_leaf(make_node_header({0.0, 0.001}, lod_bounds, tight_bounds), camera, rule));
    }
}
