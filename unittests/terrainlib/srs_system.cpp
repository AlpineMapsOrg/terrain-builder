#include <catch2/catch_test_macros.hpp>

#include <string>

#include "../result_helpers.h"
#include "error/format.h"
#include "gdal/OgrError.h"
#include "srs/System.h"

TEST_CASE("Importing an unknown EPSG code fails", "[srs][System]") {
    CHECK(REQUIRE_ERROR(srs::from_epsg(999999)).holds<gdal::OgrError>());
}

TEST_CASE("Parsing invalid spatial reference input fails", "[srs][System]") {
    const std::string input = "not a spatial reference";

    CHECK(REQUIRE_ERROR(srs::from_user_input(input)).holds<gdal::OgrError>());
}
