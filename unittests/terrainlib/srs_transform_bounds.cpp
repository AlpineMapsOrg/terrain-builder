#include <catch2/catch_test_macros.hpp>

#include <optional>

#include <radix/geometry.h>
#include <radix/tile.h>

#include "../result_helpers.h"
#include "error/format.h"
#include "srs/System.h"
#include "srs/transform.h"
#include "srs/transform_bounds.h"

// The orthographic projection onto the north pole maps no point south of the equator.
TEST_CASE("transforming fails for bounds outside the projection domain", "[srs][transform_bounds]") {
    const srs::System geographic = REQUIRE_VALUE(srs::from_user_input("EPSG:4326"));
    const srs::System orthographic = REQUIRE_VALUE(srs::from_user_input("ESRI:102035"));
    const srs::Transform transform = REQUIRE_VALUE(srs::get_transform(geographic, orthographic));

    std::optional<alp::Error<srs::Untransformable>> error_opt;
    SECTION("non_exact_bounds_transform in 2d") {
        const radix::tile::SrsBounds bounds({10, -40}, {30, 20});
        error_opt = REQUIRE_ERROR(srs::non_exact_bounds_transform(transform, bounds));
    }
    SECTION("non_exact_bounds_transform in 3d") {
        const radix::geometry::Aabb3d bounds({10, -40, 300}, {30, 20, 500});
        error_opt = REQUIRE_ERROR(srs::non_exact_bounds_transform(transform, bounds));
    }
    SECTION("encompassing_bounds_transfer in 2d") {
        const radix::tile::SrsBounds bounds({10, -40}, {30, -20});
        error_opt = REQUIRE_ERROR(srs::encompassing_bounds_transfer(transform, bounds));
    }
    SECTION("encompassing_bounds_transfer in 3d") {
        const radix::geometry::Aabb3d bounds({10, -40, 300}, {30, -20, 500});
        error_opt = REQUIRE_ERROR(srs::encompassing_bounds_transfer(transform, bounds));
    }

    const srs::Untransformable *untransformable = error_opt.value().get_if<srs::Untransformable>();
    REQUIRE(untransformable != nullptr);
    CHECK(untransformable->source == "EPSG:4326");
    CHECK(untransformable->target == "North_Pole_Orthographic");
}
