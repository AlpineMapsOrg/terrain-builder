/*****************************************************************************
 * Alpine Terrain Builder
 * Copyright (C) 2022 alpinemaps.org
 * Copyright (C) 2022 Adam Celarek <family name at cg tuwien ac at>
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

#include "../catch2_helpers.h"
#include <catch2/catch.hpp>
#include <glm/fwd.hpp>
#include <glm/glm.hpp>

#include "algorithms/primitives.h"

TEST_CASE("primitive tests")
{
    SECTION("triangle area and cw / ccw")
    {
        CHECK(primitives::triAreaX2(glm::dvec2(0, 0), glm::dvec2(10, 0), glm::dvec2(10, 10)) == Approx(100));

        CHECK(primitives::ccw(glm::dvec2(0, 0), glm::dvec2(10, 0), glm::dvec2(10, 10)));
        CHECK(!primitives::ccw(glm::dvec2(0, 0), glm::dvec2(10, 10), glm::dvec2(10, 0)));
        CHECK(primitives::ccw(glm::dvec2(0, 0), glm::dvec2(0.10, 0), glm::dvec2(0.10, 0.1)));
        CHECK(!primitives::ccw(glm::dvec2(0, 0), glm::dvec2(0.10, 10), glm::dvec2(0.10, 0)));
    }

    SECTION("left/rightOf for floating types")
    {
        CHECK(primitives::leftOf(glm::dvec2(0, 5), glm::dvec2(0, 0), glm::dvec2(5, 5)));
        CHECK(primitives::rightOf(glm::dvec2(5, 0), glm::dvec2(0, 0), glm::dvec2(5, 5)));
        CHECK(!primitives::leftOf(glm::dvec2(5, 0), glm::dvec2(0, 0), glm::dvec2(5, 5)));
        CHECK(!primitives::rightOf(glm::dvec2(0, 5), glm::dvec2(0, 0), glm::dvec2(5, 5)));
    }

    SECTION("left/rightOf for unsigned types")
    {
        CHECK(primitives::leftOf(glm::uvec2(0, 5), glm::uvec2(0, 0), glm::uvec2(5, 5)));
        CHECK(primitives::rightOf(glm::uvec2(5, 0), glm::uvec2(0, 0), glm::uvec2(5, 5)));
        CHECK(!primitives::leftOf(glm::uvec2(5, 0), glm::uvec2(0, 0), glm::uvec2(5, 5)));
        CHECK(!primitives::rightOf(glm::uvec2(0, 5), glm::uvec2(0, 0), glm::uvec2(5, 5)));
    }

    SECTION("left/rightOf for unsigned types and border cases")
    {
        CHECK(primitives::leftOf<true>(glm::uvec2(0, 5), glm::uvec2(0, 0), glm::uvec2(0, 5)));
        CHECK(primitives::rightOf<true>(glm::uvec2(3, 3), glm::uvec2(0, 0), glm::uvec2(5, 5)));
        CHECK(!primitives::leftOf<false>(glm::uvec2(0, 5), glm::uvec2(0, 0), glm::uvec2(0, 5)));
        CHECK(!primitives::rightOf<false>(glm::uvec2(3, 3), glm::uvec2(0, 0), glm::uvec2(5, 5)));
    }

    SECTION("triangle inside / outside normal cases")
    {
        CHECK(primitives::inside<unsigned>({ 2, 1 }, { 0, 0 }, { 3, 0 }, { 3, 3 }));
        CHECK(primitives::inside<unsigned>({ 2, 1 }, { 0, 0 }, { 30, 0 }, { 3, 3 }));
        CHECK(primitives::inside<unsigned>({ 2, 1 }, { 0, 0 }, { 3, 0 }, { 3, 30 }));
        CHECK(primitives::inside<unsigned>({ 2, 1 }, { 0, 0 }, { 3, 0 }, { 30, 30 }));

        CHECK(!primitives::inside<unsigned>({ 1, 2 }, { 0, 0 }, { 3, 0 }, { 3, 3 }));
        CHECK(!primitives::inside<unsigned>({ 3, 4 }, { 0, 0 }, { 30, 0 }, { 3, 3 }));
        CHECK(!primitives::inside<unsigned>({ 4, 0 }, { 0, 0 }, { 3, 0 }, { 3, 30 }));
        CHECK(!primitives::inside<unsigned>({ 2, 3 }, { 0, 0 }, { 3, 0 }, { 30, 30 }));
    }

    SECTION("bottom right triangle, edges should be inside")
    {
        CHECK(!primitives::inside<unsigned>({ 0, 0 }, { 0, 0 }, { 3, 0 }, { 3, 3 }));
        CHECK(primitives::inside<unsigned>({ 1, 1 }, { 0, 0 }, { 3, 0 }, { 3, 3 }));
        CHECK(primitives::inside<unsigned>({ 2, 2 }, { 0, 0 }, { 3, 0 }, { 3, 3 }));
        CHECK(!primitives::inside<unsigned>({ 1, 0 }, { 0, 0 }, { 3, 0 }, { 3, 3 }));
        CHECK(!primitives::inside<unsigned>({ 2, 0 }, { 0, 0 }, { 3, 0 }, { 3, 3 }));
        CHECK(!primitives::inside<unsigned>({ 3, 0 }, { 0, 0 }, { 3, 0 }, { 3, 3 }));
        CHECK(!primitives::inside<unsigned>({ 3, 1 }, { 0, 0 }, { 3, 0 }, { 3, 3 }));
        CHECK(!primitives::inside<unsigned>({ 3, 2 }, { 0, 0 }, { 3, 0 }, { 3, 3 }));
        CHECK(!primitives::inside<unsigned>({ 3, 3 }, { 0, 0 }, { 3, 0 }, { 3, 3 }));
    }

    SECTION("top left triangle, left and top should be inside, diagonal should be outside")
    {
        CHECK(!primitives::inside<unsigned>({ 0, 0 }, { 0, 0 }, { 3, 3 }, { 0, 3 }));
        CHECK(!primitives::inside<unsigned>({ 1, 1 }, { 0, 0 }, { 3, 3 }, { 0, 3 }));
        CHECK(!primitives::inside<unsigned>({ 2, 2 }, { 0, 0 }, { 3, 3 }, { 0, 3 }));
        CHECK(primitives::inside<unsigned>({ 1, 3 }, { 0, 0 }, { 3, 3 }, { 0, 3 }));
        CHECK(primitives::inside<unsigned>({ 2, 3 }, { 0, 0 }, { 3, 3 }, { 0, 3 }));
        CHECK(!primitives::inside<unsigned>({ 3, 3 }, { 0, 0 }, { 3, 3 }, { 0, 3 }));
        CHECK(primitives::inside<unsigned>({ 0, 1 }, { 0, 0 }, { 3, 3 }, { 0, 3 }));
        CHECK(primitives::inside<unsigned>({ 0, 2 }, { 0, 0 }, { 3, 3 }, { 0, 3 }));
        CHECK(primitives::inside<unsigned>({ 0, 3 }, { 0, 0 }, { 3, 3 }, { 0, 3 }));
    }
}
