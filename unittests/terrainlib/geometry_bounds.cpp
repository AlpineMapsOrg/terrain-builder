#include "../catch2_helpers.h"

#include <array>
#include <limits>
#include <vector>

#include <catch2/matchers/catch_matchers_contains.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_range_equals.hpp>
#include <glm/glm.hpp>
#include <radix/geometry.h>

#include "geometry/bounds.h"
#include "geometry/geometry.h"

TEST_CASE("is_empty holds unless min < max on every axis", "[geometry][bounds]") {
    SECTION("a box with extent on every axis") {
        CHECK_FALSE(geometry::is_empty(radix::geometry::Aabb2d({0, 0}, {1, 1})));
    }
    SECTION("a box of zero width") {
        CHECK(geometry::is_empty(radix::geometry::Aabb2d({1, 0}, {1, 1})));
    }
    SECTION("an inverted box") {
        CHECK(geometry::is_empty(radix::geometry::Aabb2d({0, 1}, {1, 0})));
    }
    SECTION("a default-constructed box") {
        CHECK(geometry::is_empty(radix::geometry::Aabb3d()));
    }
}

TEST_CASE("is_finite checks every coordinate of the bounds", "[geometry][bounds]") {
    SECTION("a finite box") {
        CHECK(geometry::is_finite(radix::geometry::Aabb2d({0, 0}, {1, 1})));
    }
    SECTION("a box with an infinite max") {
        CHECK_FALSE(geometry::is_finite(radix::geometry::Aabb2d({0, 0}, {std::numeric_limits<double>::infinity(), 1})));
    }
    SECTION("a default-constructed box, which holds the largest finite values") {
        CHECK(geometry::is_finite(radix::geometry::Aabb3d()));
    }
}

TEST_CASE("wrap_into_periods shifts bounds by whole periods into the range around the centre", "[geometry][bounds]") {
    SECTION("from above") {
        const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb2d({7, 1}, {8, 2}), glm::bvec2(true, false), glm::dvec2(0, 0), glm::dvec2(10, 0));

        CHECK(wrapped == std::vector<radix::geometry::Aabb<2, double>>{radix::geometry::Aabb2d({-3, 1}, {-2, 2})});
    }
    SECTION("from more than a period below") {
        const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb2d({-23, 1}, {-22, 2}), glm::bvec2(true, false), glm::dvec2(0, 0), glm::dvec2(10, 0));

        CHECK(wrapped == std::vector<radix::geometry::Aabb<2, double>>{radix::geometry::Aabb2d({-3, 1}, {-2, 2})});
    }
}

TEST_CASE("wrap_into_periods includes the lower end of the range and excludes the upper end", "[geometry][bounds]") {
    SECTION("bounds starting at the lower end stay") {
        const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb2d({-5, 1}, {-4, 2}), glm::bvec2(true, false), glm::dvec2(0, 0), glm::dvec2(10, 0));

        CHECK(wrapped == std::vector<radix::geometry::Aabb<2, double>>{radix::geometry::Aabb2d({-5, 1}, {-4, 2})});
    }
    SECTION("bounds starting at the upper end move to the lower end") {
        const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb2d({5, 1}, {6, 2}), glm::bvec2(true, false), glm::dvec2(0, 0), glm::dvec2(10, 0));

        CHECK(wrapped == std::vector<radix::geometry::Aabb<2, double>>{radix::geometry::Aabb2d({-5, 1}, {-4, 2})});
    }
}

TEST_CASE("wrap_into_periods splits bounds that reach past the upper end of the range", "[geometry][bounds]") {
    const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb2d({4, 1}, {7, 2}), glm::bvec2(true, false), glm::dvec2(0, 0), glm::dvec2(10, 0));

    const std::vector<radix::geometry::Aabb<2, double>> expected{radix::geometry::Aabb2d({4, 1}, {5, 2}), radix::geometry::Aabb2d({-5, 1}, {-3, 2})};
    CHECK_THAT(wrapped, Catch::Matchers::UnorderedRangeEquals(expected));
}

TEST_CASE("wrap_into_periods reads max below min as running across the end of a period", "[geometry][bounds]") {
    const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb2d({4, 1}, {-4, 2}), glm::bvec2(true, false), glm::dvec2(0, 0), glm::dvec2(10, 0));

    const std::vector<radix::geometry::Aabb<2, double>> expected{radix::geometry::Aabb2d({4, 1}, {5, 2}), radix::geometry::Aabb2d({-5, 1}, {-4, 2})};
    CHECK_THAT(wrapped, Catch::Matchers::UnorderedRangeEquals(expected));
}

TEST_CASE("wrap_into_periods gives the whole range for bounds spanning a period", "[geometry][bounds]") {
    const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb2d({-2, 1}, {8, 2}), glm::bvec2(true, false), glm::dvec2(0, 0), glm::dvec2(10, 0));

    CHECK(wrapped == std::vector<radix::geometry::Aabb<2, double>>{radix::geometry::Aabb2d({-5, 1}, {5, 2})});
}

TEST_CASE("wrap_into_periods wraps only the dimensions it is told to", "[geometry][bounds]") {
    SECTION("2d") {
        const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb2d({30, 107}, {40, 108}), glm::bvec2(false, true), glm::dvec2(0, 100), glm::dvec2(0, 10));

        CHECK(wrapped == std::vector<radix::geometry::Aabb<2, double>>{radix::geometry::Aabb2d({30, 97}, {40, 98})});
    }
    SECTION("3d") {
        const std::vector<radix::geometry::Aabb3d> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb3d({30, 40, 107}, {31, 41, 108}), glm::bvec3(false, false, true), glm::dvec3(0, 0, 100), glm::dvec3(0, 0, 10));

        CHECK(wrapped == std::vector{radix::geometry::Aabb3d({30, 40, 97}, {31, 41, 98})});
    }
}

TEST_CASE("wrap_into_periods splits along each dimension it wraps", "[geometry][bounds]") {
    const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb2d({4, 14}, {7, 17}), glm::bvec2(true, true), glm::dvec2(0, 10), glm::dvec2(10, 10));

    const std::vector<radix::geometry::Aabb<2, double>> expected{
        radix::geometry::Aabb2d({4, 14}, {5, 15}),
        radix::geometry::Aabb2d({-5, 14}, {-3, 15}),
        radix::geometry::Aabb2d({4, 5}, {5, 7}),
        radix::geometry::Aabb2d({-5, 5}, {-3, 7}),
    };
    CHECK_THAT(wrapped, Catch::Matchers::UnorderedRangeEquals(expected));
}

TEST_CASE("get_corner_in_direction gets the corner the direction points towards", "[geometry][bounds]") {
    SECTION("2d") {
        const radix::geometry::Aabb2d bounds{{1.0, 2.0}, {4.0, 6.0}};

        CHECK(geometry::get_corner_in_direction(bounds, glm::dvec2(0.5, 2.0)) == glm::dvec2(4.0, 6.0));
        CHECK(geometry::get_corner_in_direction(bounds, glm::dvec2(0.5, -2.0)) == glm::dvec2(4.0, 2.0));
        CHECK(geometry::get_corner_in_direction(bounds, glm::dvec2(-0.5, 2.0)) == glm::dvec2(1.0, 6.0));
        CHECK(geometry::get_corner_in_direction(bounds, glm::dvec2(-0.5, -2.0)) == glm::dvec2(1.0, 2.0));
    }
    SECTION("3d") {
        const radix::geometry::Aabb3d bounds{{1.0, 2.0, 3.0}, {3.0, 5.0, 9.0}};

        CHECK(geometry::get_corner_in_direction(bounds, glm::dvec3(0.5, -2.0, 1.0)) == glm::dvec3(3.0, 2.0, 9.0));
        CHECK(geometry::get_corner_in_direction(bounds, glm::dvec3(-0.5, 2.0, -1.0)) == glm::dvec3(1.0, 5.0, 3.0));
    }
}

TEST_CASE("get_corner_in_direction gets one of the corners that reach equally far", "[geometry][bounds]") {
    SECTION("2d") {
        const radix::geometry::Aabb2d bounds{{1.0, 2.0}, {4.0, 6.0}};

        SECTION("along an edge") {
            const std::array farthest{glm::dvec2(1.0, 6.0), glm::dvec2(4.0, 6.0)};
            CHECK_THAT(farthest, Catch::Matchers::Contains(geometry::get_corner_in_direction(bounds, glm::dvec2(0.0, 1.0))));
        }
        SECTION("without a direction") {
            const std::array corners{glm::dvec2(1.0, 2.0), glm::dvec2(4.0, 2.0), glm::dvec2(1.0, 6.0), glm::dvec2(4.0, 6.0)};
            CHECK_THAT(corners, Catch::Matchers::Contains(geometry::get_corner_in_direction(bounds, glm::dvec2(0.0, 0.0))));
        }
    }
    SECTION("3d") {
        const radix::geometry::Aabb3d bounds{{1.0, 2.0, 3.0}, {3.0, 5.0, 9.0}};

        SECTION("along an edge") {
            const std::array farthest{glm::dvec3(1.0, 2.0, 9.0), glm::dvec3(3.0, 2.0, 9.0)};
            CHECK_THAT(farthest, Catch::Matchers::Contains(geometry::get_corner_in_direction(bounds, glm::dvec3(0.0, -1.0, 1.0))));
        }
        SECTION("without a direction") {
            CHECK_THAT(radix::geometry::corners(bounds), Catch::Matchers::Contains(geometry::get_corner_in_direction(bounds, glm::dvec3(0.0, 0.0, 0.0))));
        }
    }
}

TEST_CASE("find_nearest_point_in_bounds keeps a point that is in the bounds", "[geometry][bounds]") {
    SECTION("2d") {
        const radix::geometry::Aabb2d bounds{{1.0, 2.0}, {4.0, 6.0}};

        SECTION("in the interior") {
            CHECK(geometry::find_nearest_point_in_bounds(bounds, glm::dvec2(2.1, 3.1)) == glm::dvec2(2.1, 3.1));
        }
        SECTION("on an edge") {
            CHECK(geometry::find_nearest_point_in_bounds(bounds, glm::dvec2(4.0, 3.1)) == glm::dvec2(4.0, 3.1));
        }
    }
    SECTION("3d") {
        const radix::geometry::Aabb3d bounds{{1.0, 2.0, 3.0}, {3.0, 5.0, 9.0}};

        SECTION("in the interior") {
            CHECK(geometry::find_nearest_point_in_bounds(bounds, glm::dvec3(2.1, 3.1, 4.1)) == glm::dvec3(2.1, 3.1, 4.1));
        }
        SECTION("on a face") {
            CHECK(geometry::find_nearest_point_in_bounds(bounds, glm::dvec3(3.0, 3.1, 4.1)) == glm::dvec3(3.0, 3.1, 4.1));
        }
    }
}

TEST_CASE("find_nearest_point_in_bounds moves a point outside the bounds onto them", "[geometry][bounds]") {
    SECTION("2d") {
        const radix::geometry::Aabb2d bounds{{1.0, 2.0}, {4.0, 6.0}};

        SECTION("past an edge") {
            CHECK(geometry::find_nearest_point_in_bounds(bounds, glm::dvec2(5.0, 3.1)) == glm::dvec2(4.0, 3.1));
        }
        SECTION("past a corner") {
            CHECK(geometry::find_nearest_point_in_bounds(bounds, glm::dvec2(0.0, 7.0)) == glm::dvec2(1.0, 6.0));
        }
    }
    SECTION("3d") {
        const radix::geometry::Aabb3d bounds{{1.0, 2.0, 3.0}, {3.0, 5.0, 9.0}};

        SECTION("past a face") {
            CHECK(geometry::find_nearest_point_in_bounds(bounds, glm::dvec3(4.0, 3.1, 4.1)) == glm::dvec3(3.0, 3.1, 4.1));
        }
        SECTION("past an edge") {
            CHECK(geometry::find_nearest_point_in_bounds(bounds, glm::dvec3(4.0, 6.0, 4.1)) == glm::dvec3(3.0, 5.0, 4.1));
        }
        SECTION("past a corner") {
            CHECK(geometry::find_nearest_point_in_bounds(bounds, glm::dvec3(0.0, 1.0, 10.0)) == glm::dvec3(1.0, 2.0, 9.0));
        }
    }
}

TEST_CASE("find_farthest_point_in_bounds finds the corner across the centre from the point", "[geometry][bounds]") {
    SECTION("2d") {
        // The centre is (2.5, 4).
        const radix::geometry::Aabb2d bounds{{1.0, 2.0}, {4.0, 6.0}};

        SECTION("from inside") {
            CHECK(geometry::find_farthest_point_in_bounds(bounds, glm::dvec2(2.0, 5.0)) == glm::dvec2(4.0, 2.0));
        }
        SECTION("from outside") {
            CHECK(geometry::find_farthest_point_in_bounds(bounds, glm::dvec2(5.0, 0.0)) == glm::dvec2(1.0, 6.0));
        }
    }
    SECTION("3d") {
        // The centre is (2, 3.5, 6).
        const radix::geometry::Aabb3d bounds{{1.0, 2.0, 3.0}, {3.0, 5.0, 9.0}};

        SECTION("from inside") {
            CHECK(geometry::find_farthest_point_in_bounds(bounds, glm::dvec3(1.5, 4.0, 5.0)) == glm::dvec3(3.0, 2.0, 9.0));
        }
        SECTION("from outside") {
            CHECK(geometry::find_farthest_point_in_bounds(bounds, glm::dvec3(4.0, 0.0, 10.0)) == glm::dvec3(1.0, 5.0, 3.0));
        }
    }
}

TEST_CASE("find_farthest_point_in_bounds finds one of the corners that are equally far", "[geometry][bounds]") {
    SECTION("2d") {
        // The centre is (2.5, 4).
        const radix::geometry::Aabb2d bounds{{1.0, 2.0}, {4.0, 6.0}};

        SECTION("level with the centre along x") {
            const std::array farthest{glm::dvec2(1.0, 2.0), glm::dvec2(4.0, 2.0)};
            CHECK_THAT(farthest, Catch::Matchers::Contains(geometry::find_farthest_point_in_bounds(bounds, glm::dvec2(2.5, 5.0))));
        }
        SECTION("from the centre") {
            const std::array corners{glm::dvec2(1.0, 2.0), glm::dvec2(4.0, 2.0), glm::dvec2(1.0, 6.0), glm::dvec2(4.0, 6.0)};
            CHECK_THAT(corners, Catch::Matchers::Contains(geometry::find_farthest_point_in_bounds(bounds, glm::dvec2(2.5, 4.0))));
        }
    }
    SECTION("3d") {
        // The centre is (2, 3.5, 6).
        const radix::geometry::Aabb3d bounds{{1.0, 2.0, 3.0}, {3.0, 5.0, 9.0}};

        SECTION("level with the centre along x") {
            const std::array farthest{glm::dvec3(1.0, 2.0, 9.0), glm::dvec3(3.0, 2.0, 9.0)};
            CHECK_THAT(farthest, Catch::Matchers::Contains(geometry::find_farthest_point_in_bounds(bounds, glm::dvec3(2.0, 4.0, 5.0))));
        }
        SECTION("from the centre") {
            CHECK_THAT(radix::geometry::corners(bounds), Catch::Matchers::Contains(geometry::find_farthest_point_in_bounds(bounds, glm::dvec3(2.0, 3.5, 6.0))));
        }
    }
}

TEST_CASE("compute_nearest_distance_to_bounds measures to the nearest point of the bounds", "[geometry][bounds]") {
    SECTION("2d") {
        const radix::geometry::Aabb2d bounds{{1.0, 2.0}, {4.0, 6.0}};

        SECTION("from inside") {
            CHECK(geometry::compute_nearest_distance_to_bounds(bounds, glm::dvec2(2.1, 3.1)) == 0.0);
        }
        SECTION("from outside") {
            // The nearest point is (4, 6), which is 3 and 4 away along the axes.
            CHECK_THAT(geometry::compute_nearest_distance_to_bounds(bounds, glm::dvec2(7.0, 10.0)), Catch::Matchers::WithinAbs(5.0, geometry::epsilon<double>()));
        }
    }
    SECTION("3d") {
        const radix::geometry::Aabb3d bounds{{1.0, 2.0, 3.0}, {3.0, 5.0, 9.0}};

        SECTION("from inside") {
            CHECK(geometry::compute_nearest_distance_to_bounds(bounds, glm::dvec3(2.1, 3.1, 4.1)) == 0.0);
        }
        SECTION("from outside") {
            // The nearest point is (3, 5, 9), which is 2, 3 and 6 away along the axes.
            CHECK_THAT(geometry::compute_nearest_distance_to_bounds(bounds, glm::dvec3(5.0, 8.0, 15.0)), Catch::Matchers::WithinAbs(7.0, geometry::epsilon<double>()));
        }
    }
}

TEST_CASE("compute_farthest_distance_to_bounds measures to the farthest corner of the bounds", "[geometry][bounds]") {
    SECTION("2d") {
        // The farthest corner from (1, 2) is (4, 6), which is 3 and 4 away along the axes.
        const radix::geometry::Aabb2d bounds{{1.0, 2.0}, {4.0, 6.0}};

        CHECK_THAT(geometry::compute_farthest_distance_to_bounds(bounds, glm::dvec2(1.0, 2.0)), Catch::Matchers::WithinAbs(5.0, geometry::epsilon<double>()));
    }
    SECTION("3d") {
        // The farthest corner from (1, 2, 3) is (3, 5, 9), which is 2, 3 and 6 away along the axes.
        const radix::geometry::Aabb3d bounds{{1.0, 2.0, 3.0}, {3.0, 5.0, 9.0}};

        CHECK_THAT(geometry::compute_farthest_distance_to_bounds(bounds, glm::dvec3(1.0, 2.0, 3.0)), Catch::Matchers::WithinAbs(7.0, geometry::epsilon<double>()));
    }
}

TEST_CASE("relate_to_plane relates bounds to a plane facing +x", "[geometry][bounds]") {
    const radix::geometry::Aabb3d bounds{{1.0, 2.0, 3.0}, {3.0, 5.0, 9.0}};
    using Plane = radix::geometry::Plane<double>;

    SECTION("in front of the plane x = 0") {
        CHECK(geometry::relate_to_plane(bounds, Plane(glm::dvec3(1.0, 0.0, 0.0), 0.0)) == geometry::PlaneRelation::Front);
    }
    SECTION("intersecting the plane x = 2") {
        CHECK(geometry::relate_to_plane(bounds, Plane(glm::dvec3(1.0, 0.0, 0.0), -2.0)) == geometry::PlaneRelation::Intersecting);
    }
    SECTION("behind the plane x = 4") {
        CHECK(geometry::relate_to_plane(bounds, Plane(glm::dvec3(1.0, 0.0, 0.0), -4.0)) == geometry::PlaneRelation::Back);
    }
}

TEST_CASE("relate_to_plane counts bounds touching a plane as intersecting it", "[geometry][bounds]") {
    const radix::geometry::Aabb3d bounds{{1.0, 2.0, 3.0}, {3.0, 5.0, 9.0}};
    using Plane = radix::geometry::Plane<double>;

    SECTION("touching the plane x = 1 from in front") {
        CHECK(geometry::relate_to_plane(bounds, Plane(glm::dvec3(1.0, 0.0, 0.0), -1.0)) == geometry::PlaneRelation::Intersecting);
    }
    SECTION("touching the plane x = 3 from behind") {
        CHECK(geometry::relate_to_plane(bounds, Plane(glm::dvec3(1.0, 0.0, 0.0), -3.0)) == geometry::PlaneRelation::Intersecting);
    }
}

TEST_CASE("relate_to_plane relates bounds to a slanted plane", "[geometry][bounds]") {
    // Along the normal (0.6, 0.8, 0) the corners at x = 1, y = 2 reach 2.2 and the ones at x = 3, y = 5 reach 5.8.
    // Along the normal (0, 0.6, 0.8) the corners at y = 2, z = 3 reach 3.6 and the ones at y = 5, z = 9 reach 10.2.
    const radix::geometry::Aabb3d bounds{{1.0, 2.0, 3.0}, {3.0, 5.0, 9.0}};
    using Plane = radix::geometry::Plane<double>;

    SECTION("in front") {
        CHECK(geometry::relate_to_plane(bounds, Plane(glm::dvec3(0.6, 0.8, 0.0), -2.0)) == geometry::PlaneRelation::Front);
        CHECK(geometry::relate_to_plane(bounds, Plane(glm::dvec3(-0.6, -0.8, 0.0), 6.0)) == geometry::PlaneRelation::Front);
        CHECK(geometry::relate_to_plane(bounds, Plane(glm::dvec3(0.0, 0.6, 0.8), -3.0)) == geometry::PlaneRelation::Front);
    }
    SECTION("intersecting") {
        CHECK(geometry::relate_to_plane(bounds, Plane(glm::dvec3(0.6, 0.8, 0.0), -4.0)) == geometry::PlaneRelation::Intersecting);
        CHECK(geometry::relate_to_plane(bounds, Plane(glm::dvec3(-0.6, -0.8, 0.0), 4.0)) == geometry::PlaneRelation::Intersecting);
        CHECK(geometry::relate_to_plane(bounds, Plane(glm::dvec3(0.0, 0.6, 0.8), -5.0)) == geometry::PlaneRelation::Intersecting);
    }
    SECTION("behind") {
        CHECK(geometry::relate_to_plane(bounds, Plane(glm::dvec3(0.6, 0.8, 0.0), -6.0)) == geometry::PlaneRelation::Back);
        CHECK(geometry::relate_to_plane(bounds, Plane(glm::dvec3(-0.6, -0.8, 0.0), 2.0)) == geometry::PlaneRelation::Back);
        CHECK(geometry::relate_to_plane(bounds, Plane(glm::dvec3(0.0, 0.6, 0.8), -10.5)) == geometry::PlaneRelation::Back);
    }
}
