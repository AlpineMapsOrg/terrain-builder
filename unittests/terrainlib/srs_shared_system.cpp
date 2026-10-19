#include <catch2/catch_test_macros.hpp>

#include <thread>

#include "../result_helpers.h"
#include "srs/System.h"
#include "srs/SharedSystem.h"

TEST_CASE("SharedSystem::get gives a copy of the system it was made from", "[srs][SharedSystem]") {
    const srs::System mercator = REQUIRE_VALUE(srs::from_epsg(3857));
    const srs::SharedSystem shared(mercator);

    CHECK(shared.get().IsSame(&mercator));
}

TEST_CASE("SharedSystem::get gives the same copy on every call from one thread", "[srs][SharedSystem]") {
    const srs::SharedSystem shared(REQUIRE_VALUE(srs::from_epsg(3857)));

    const srs::System *first = &shared.get();
    const srs::System *second = &shared.get();

    CHECK(first == second);
}

TEST_CASE("SharedSystem::get gives each thread its own copy", "[srs][SharedSystem]") {
    const srs::SharedSystem shared(REQUIRE_VALUE(srs::from_epsg(3857)));

    const srs::System *main_copy = &shared.get();
    const srs::System *other_copy = nullptr;
    std::thread([&] {
        other_copy = &shared.get();
    }).join();

    CHECK(main_copy != other_copy);
}
