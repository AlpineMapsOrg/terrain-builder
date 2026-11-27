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

#include <catch2/catch_test_macros.hpp>

#include <thread>

#include "../result_helpers.h"
#include "gis/srs/System.h"
#include "gis/srs/ThreadLocalSystem.h"

TEST_CASE("ThreadLocalSystem::get gives a copy of the system it was made from", "[gis][srs][ThreadLocalSystem]") {
    const gis::srs::System mercator = REQUIRE_VALUE(gis::srs::from_epsg(3857));
    const gis::srs::ThreadLocalSystem shared(mercator);

    CHECK(shared.get().is_same(mercator));
}

TEST_CASE("ThreadLocalSystem::get gives the same copy on every call from one thread", "[gis][srs][ThreadLocalSystem]") {
    const gis::srs::ThreadLocalSystem shared(REQUIRE_VALUE(gis::srs::from_epsg(3857)));

    const gis::srs::System *first = &shared.get();
    const gis::srs::System *second = &shared.get();

    CHECK(first == second);
}

TEST_CASE("ThreadLocalSystem::get gives each thread its own copy", "[gis][srs][ThreadLocalSystem]") {
    const gis::srs::ThreadLocalSystem shared(REQUIRE_VALUE(gis::srs::from_epsg(3857)));

    const gis::srs::System *main_copy = &shared.get();
    const gis::srs::System *other_copy = nullptr;
    std::thread([&] {
        other_copy = &shared.get();
    }).join();

    CHECK(&main_copy->as_gdal() != &other_copy->as_gdal());
}
