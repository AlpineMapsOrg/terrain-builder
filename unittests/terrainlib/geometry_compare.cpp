#include "../catch2_helpers.h"

#include <array>
#include <compare>
#include <limits>
#include <vector>

#include <glm/glm.hpp>
#include <radix/geometry.h>

#include "geometry/compare.h"

TEST_CASE("compare_strong orders vectors by the first component that differs", "[geometry][compare]") {
    const glm::dvec3 vector(1.0, 5.0, 9.0);

    CHECK(geometry::compare_strong(vector, glm::dvec3(2.0, 0.0, 0.0)) == std::strong_ordering::less);
    CHECK(geometry::compare_strong(vector, glm::dvec3(1.0, 4.0, 10.0)) == std::strong_ordering::greater);
    CHECK(geometry::compare_strong(vector, glm::dvec3(1.0, 5.0, 9.5)) == std::strong_ordering::less);
    CHECK(geometry::compare_strong(vector, glm::dvec3(1.0, 5.0, 9.0)) == std::strong_ordering::equal);

    CHECK(geometry::compare_strong(glm::ivec2(3, 7), glm::ivec2(3, 6)) == std::strong_ordering::greater);
}

TEST_CASE("compare_strong gives nan and signed zeros a place", "[geometry][compare]") {
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    constexpr double infinity = std::numeric_limits<double>::infinity();

    CHECK(geometry::compare_strong(glm::dvec2(1.0, nan), glm::dvec2(1.0, nan)) == std::strong_ordering::equal);
    CHECK(geometry::compare_strong(glm::dvec2(1.0, infinity), glm::dvec2(1.0, nan)) == std::strong_ordering::less);
    CHECK(geometry::compare_strong(glm::dvec2(-0.0, 1.0), glm::dvec2(0.0, 1.0)) == std::strong_ordering::less);
}

TEST_CASE("compare_strong orders ranges by the first element that differs", "[geometry][compare]") {
    const std::vector<int> range = {1, 5, 9};

    CHECK(geometry::compare_strong(range, std::vector<int>{2, 0, 0}) == std::strong_ordering::less);
    CHECK(geometry::compare_strong(range, std::vector<int>{1, 5, 8}) == std::strong_ordering::greater);
    CHECK(geometry::compare_strong(range, std::vector<int>{1, 5, 9}) == std::strong_ordering::equal);

    // A range that is the start of the other comes first.
    CHECK(geometry::compare_strong(range, std::vector<int>{1, 5, 9, 0}) == std::strong_ordering::less);
    CHECK(geometry::compare_strong(range, std::vector<int>{1, 5}) == std::strong_ordering::greater);

    // The elements compare with compare_strong themselves.
    const std::array<glm::dvec2, 2> vectors = {glm::dvec2(1.0, 2.0), glm::dvec2(3.0, 4.0)};
    CHECK(geometry::compare_strong(vectors, std::array<glm::dvec2, 2>{glm::dvec2(1.0, 2.0), glm::dvec2(3.0, 4.5)}) == std::strong_ordering::less);
}

TEST_CASE("Strong compares its value in the order of compare_strong", "[geometry][compare]") {
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    CHECK(geometry::Strong{nan} == geometry::Strong{nan});
    CHECK(geometry::Strong{-0.0} < geometry::Strong{0.0});
    CHECK(geometry::Strong{glm::dvec2(1.0, 3.0)} > geometry::Strong{glm::dvec2(1.0, 2.0)});
}

TEST_CASE("tie_strong compares the given values in order", "[geometry][compare]") {
    const double low = 1.0;
    const double high = 2.0;
    const glm::dvec2 small(0.0, 0.0);
    const glm::dvec2 large(5.0, 5.0);

    // The first value decides even where the second says the opposite.
    CHECK(geometry::tie_strong(low, large) < geometry::tie_strong(high, small));
    CHECK(geometry::tie_strong(low, small) < geometry::tie_strong(low, large));
    CHECK(geometry::tie_strong(low, large) == geometry::tie_strong(low, large));
}

TEST_CASE("compare_strong orders boxes by their minimum corner, then by their maximum corner", "[geometry][compare]") {
    using Box = radix::geometry::Aabb2d;
    const Box box{glm::dvec2(1.0, 2.0), glm::dvec2(5.0, 6.0)};

    // The minimum corner decides even where the maximum corner says the opposite.
    CHECK(geometry::compare_strong<geometry::MinThenMax>(box, Box{glm::dvec2(1.0, 3.0), glm::dvec2(0.0, 0.0)}) == std::strong_ordering::less);
    CHECK(geometry::compare_strong<geometry::MinThenMax>(box, Box{glm::dvec2(0.0, 9.0), glm::dvec2(9.0, 9.0)}) == std::strong_ordering::greater);

    CHECK(geometry::compare_strong<geometry::MinThenMax>(box, Box{glm::dvec2(1.0, 2.0), glm::dvec2(5.0, 7.0)}) == std::strong_ordering::less);
    CHECK(geometry::compare_strong<geometry::MinThenMax>(box, Box{glm::dvec2(1.0, 2.0), glm::dvec2(4.0, 8.0)}) == std::strong_ordering::greater);
    CHECK(geometry::compare_strong<geometry::MinThenMax>(box, Box{glm::dvec2(1.0, 2.0), glm::dvec2(5.0, 6.0)}) == std::strong_ordering::equal);
}
