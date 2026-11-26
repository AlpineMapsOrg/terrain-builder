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

#include <array>
#include <limits>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_contains.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_range_equals.hpp>
#include <glm/glm.hpp>
#include <radix/geometry.h>

#include "../catch2_helpers.h"
#include "geometry/geometry.h"

TEST_CASE("is_empty holds unless min < max on every axis", "[geometry]") {
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

TEST_CASE("is_finite checks every coordinate of the bounds", "[geometry]") {
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

TEST_CASE("wrap_into_periods shifts bounds by whole periods into the range around the centre", "[geometry]") {
    SECTION("from above") {
        const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb2d({7, 1}, {8, 2}), glm::bvec2(true, false), glm::dvec2(0, 0), glm::dvec2(10, 0));

        CHECK(wrapped == std::vector<radix::geometry::Aabb<2, double>>{radix::geometry::Aabb2d({-3, 1}, {-2, 2})});
    }
    SECTION("from more than a period below") {
        const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb2d({-23, 1}, {-22, 2}), glm::bvec2(true, false), glm::dvec2(0, 0), glm::dvec2(10, 0));

        CHECK(wrapped == std::vector<radix::geometry::Aabb<2, double>>{radix::geometry::Aabb2d({-3, 1}, {-2, 2})});
    }
}

TEST_CASE("wrap_into_periods includes the lower end of the range and excludes the upper end", "[geometry]") {
    SECTION("bounds starting at the lower end stay") {
        const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb2d({-5, 1}, {-4, 2}), glm::bvec2(true, false), glm::dvec2(0, 0), glm::dvec2(10, 0));

        CHECK(wrapped == std::vector<radix::geometry::Aabb<2, double>>{radix::geometry::Aabb2d({-5, 1}, {-4, 2})});
    }
    SECTION("bounds starting at the upper end move to the lower end") {
        const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb2d({5, 1}, {6, 2}), glm::bvec2(true, false), glm::dvec2(0, 0), glm::dvec2(10, 0));

        CHECK(wrapped == std::vector<radix::geometry::Aabb<2, double>>{radix::geometry::Aabb2d({-5, 1}, {-4, 2})});
    }
}

TEST_CASE("wrap_into_periods splits bounds that reach past the upper end of the range", "[geometry]") {
    const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb2d({4, 1}, {7, 2}), glm::bvec2(true, false), glm::dvec2(0, 0), glm::dvec2(10, 0));

    const std::vector<radix::geometry::Aabb<2, double>> expected{radix::geometry::Aabb2d({4, 1}, {5, 2}), radix::geometry::Aabb2d({-5, 1}, {-3, 2})};
    CHECK_THAT(wrapped, Catch::Matchers::UnorderedRangeEquals(expected));
}

TEST_CASE("wrap_into_periods reads max below min as running across the end of a period", "[geometry]") {
    const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb2d({4, 1}, {-4, 2}), glm::bvec2(true, false), glm::dvec2(0, 0), glm::dvec2(10, 0));

    const std::vector<radix::geometry::Aabb<2, double>> expected{radix::geometry::Aabb2d({4, 1}, {5, 2}), radix::geometry::Aabb2d({-5, 1}, {-4, 2})};
    CHECK_THAT(wrapped, Catch::Matchers::UnorderedRangeEquals(expected));
}

TEST_CASE("wrap_into_periods gives the whole range for bounds spanning a period", "[geometry]") {
    const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb2d({-2, 1}, {8, 2}), glm::bvec2(true, false), glm::dvec2(0, 0), glm::dvec2(10, 0));

    CHECK(wrapped == std::vector<radix::geometry::Aabb<2, double>>{radix::geometry::Aabb2d({-5, 1}, {5, 2})});
}

TEST_CASE("wrap_into_periods wraps only the dimensions it is told to", "[geometry]") {
    SECTION("2d") {
        const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb2d({30, 107}, {40, 108}), glm::bvec2(false, true), glm::dvec2(0, 100), glm::dvec2(0, 10));

        CHECK(wrapped == std::vector<radix::geometry::Aabb<2, double>>{radix::geometry::Aabb2d({30, 97}, {40, 98})});
    }
    SECTION("3d") {
        const std::vector<radix::geometry::Aabb3d> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb3d({30, 40, 107}, {31, 41, 108}), glm::bvec3(false, false, true), glm::dvec3(0, 0, 100), glm::dvec3(0, 0, 10));

        CHECK(wrapped == std::vector{radix::geometry::Aabb3d({30, 40, 97}, {31, 41, 98})});
    }
}

TEST_CASE("wrap_into_periods splits along each dimension it wraps", "[geometry]") {
    const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(radix::geometry::Aabb2d({4, 14}, {7, 17}), glm::bvec2(true, true), glm::dvec2(0, 10), glm::dvec2(10, 10));

    const std::vector<radix::geometry::Aabb<2, double>> expected{
        radix::geometry::Aabb2d({4, 14}, {5, 15}),
        radix::geometry::Aabb2d({-5, 14}, {-3, 15}),
        radix::geometry::Aabb2d({4, 5}, {5, 7}),
        radix::geometry::Aabb2d({-5, 5}, {-3, 7}),
    };
    CHECK_THAT(wrapped, Catch::Matchers::UnorderedRangeEquals(expected));
}

TEST_CASE("get_corner_in_direction gets the corner the direction points towards", "[geometry]") {
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

TEST_CASE("get_corner_in_direction gets one of the corners that reach equally far", "[geometry]") {
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

TEST_CASE("find_farthest_point_in_bounds finds the corner across the centre from the point", "[geometry]") {
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

TEST_CASE("find_farthest_point_in_bounds finds one of the corners that are equally far", "[geometry]") {
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

TEST_CASE("compute_farthest_distance_to_bounds measures to the farthest corner of the bounds", "[geometry]") {
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
