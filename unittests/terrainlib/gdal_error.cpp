#include <catch2/catch_test_macros.hpp>

#include <optional>

#include <cpl_error.h>

#include "gdal/CplError.h"

TEST_CASE("A GDAL error from the last error takes what GDAL recorded", "[gdal]") {
    SECTION("an error was recorded") {
        CPLError(CE_Failure, CPLE_OpenFailed, "boom");

        const gdal::CplError error = gdal::CplError::get_last_error();

        CHECK(error.code() == CPLE_OpenFailed);
        CHECK(error.message() == "boom");
    }

    SECTION("nothing was recorded") {
        CPLErrorReset();

        const gdal::CplError error = gdal::CplError::get_last_error();

        CHECK(error.code() == CPLE_AppDefined);
        CHECK(!error.message().empty());
    }
}

TEST_CASE("try_get_last_error returns the error GDAL recorded, if it recorded one", "[gdal]") {
    SECTION("an error was recorded") {
        CPLError(CE_Failure, CPLE_OpenFailed, "boom");

        const std::optional<gdal::CplError> error_opt = gdal::CplError::try_get_last_error();

        REQUIRE(error_opt.has_value());
        CHECK(error_opt->code() == CPLE_OpenFailed);
        CHECK(error_opt->message() == "boom");
    }

    SECTION("nothing was recorded") {
        CPLErrorReset();

        CHECK(gdal::CplError::try_get_last_error() == std::nullopt);
    }
}
