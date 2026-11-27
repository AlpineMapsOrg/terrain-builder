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

#include <optional>

#include <cpl_error.h>

#include "gis/CplError.h"

TEST_CASE("A GDAL error from the last error takes what GDAL recorded", "[gis][CplError]") {
    SECTION("an error was recorded") {
        CPLError(CE_Failure, CPLE_OpenFailed, "boom");

        const gis::CplError error = gis::CplError::get_last_error();

        CHECK(error.code() == CPLE_OpenFailed);
        CHECK(error.message() == "boom");
    }

    SECTION("nothing was recorded") {
        CPLErrorReset();

        const gis::CplError error = gis::CplError::get_last_error();

        CHECK(error.code() == CPLE_AppDefined);
        CHECK(!error.message().empty());
    }
}

TEST_CASE("try_get_last_error returns the error GDAL recorded, if it recorded one", "[gis][CplError]") {
    SECTION("an error was recorded") {
        CPLError(CE_Failure, CPLE_OpenFailed, "boom");

        const std::optional<gis::CplError> error_opt = gis::CplError::try_get_last_error();

        REQUIRE(error_opt.has_value());
        CHECK(error_opt->code() == CPLE_OpenFailed);
        CHECK(error_opt->message() == "boom");
    }

    SECTION("nothing was recorded") {
        CPLErrorReset();

        CHECK(gis::CplError::try_get_last_error() == std::nullopt);
    }
}
