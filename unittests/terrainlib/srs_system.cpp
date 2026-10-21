#include <catch2/catch_test_macros.hpp>

#include <ranges>
#include <string>
#include <vector>

#include <ogr_spatialref.h>

#include "../result_helpers.h"
#include "error/format.h"
#include "gdal/OgrError.h"
#include "srs/Axis.h"
#include "srs/System.h"
#include "srs/error.h"

TEST_CASE("from_gdal fails for an srs that is not geodetic", "[srs][System]") {
    OGRSpatialReference input;
    SECTION("vertical") {
        REQUIRE(input.importFromEPSG(5778) == OGRERR_NONE);
    }
    SECTION("engineering") {
        REQUIRE(input.SetFromUserInput(R"(LOCAL_CS["test local",LOCAL_DATUM["test datum",32767],UNIT["metre",1],AXIS["x",EAST],AXIS["y",NORTH]])") == OGRERR_NONE);
    }

    CHECK(REQUIRE_ERROR(srs::from_gdal(input)).holds(srs::UnsupportedSystem::NotGeodetic));
}

TEST_CASE("from_gdal fails for an srs whose data axes are latitude first", "[srs][System]") {
    OGRSpatialReference geographic;
    REQUIRE(geographic.importFromEPSG(4326) == OGRERR_NONE);

    CHECK(REQUIRE_ERROR(srs::from_gdal(geographic)).holds(srs::UnsupportedSystem::AxisOrder));
}

TEST_CASE("from_gdal accepts every geodetic kind with its data axes in traditional order", "[srs][System]") {
    OGRSpatialReference input;
    SECTION("geographic") {
        REQUIRE(input.importFromEPSG(4326) == OGRERR_NONE);
        input.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    }
    SECTION("projected, whose authority order is easting first") {
        REQUIRE(input.importFromEPSG(3857) == OGRERR_NONE);
    }
    SECTION("geocentric") {
        REQUIRE(input.importFromEPSG(4978) == OGRERR_NONE);
    }
    SECTION("compound") {
        REQUIRE(input.SetFromUserInput("EPSG:31287+5778") == OGRERR_NONE);
        input.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    }

    CHECK(srs::from_gdal(input).has_value());
}

TEST_CASE("from_epsg fails for an unknown code", "[srs][System]") {
    CHECK(REQUIRE_ERROR(srs::from_epsg(999999)).holds<gdal::OgrError>());
}

TEST_CASE("from_epsg gives a system with its data axes in traditional order", "[srs][System]") {
    const srs::System geographic = REQUIRE_VALUE(srs::from_epsg(4326));

    CHECK(geographic.as_gdal().GetDataAxisToSRSAxisMapping() == std::vector<int>{2, 1});
}

TEST_CASE("from_epsg fails for a system that is not geodetic", "[srs][System]") {
    CHECK(REQUIRE_ERROR(srs::from_epsg(5778)).holds(srs::UnsupportedSystem::NotGeodetic));
}

TEST_CASE("from_user_input fails for invalid input", "[srs][System]") {
    CHECK(REQUIRE_ERROR(srs::from_user_input("not a spatial reference")).holds<gdal::OgrError>());
}

TEST_CASE("from_user_input gives a system with its data axes in traditional order", "[srs][System]") {
    const srs::System geographic = REQUIRE_VALUE(srs::from_user_input("EPSG:4326"));

    CHECK(geographic.as_gdal().GetDataAxisToSRSAxisMapping() == std::vector<int>{2, 1});
}

TEST_CASE("from_user_input fails for a system that is not geodetic", "[srs][System]") {
    CHECK(REQUIRE_ERROR(srs::from_user_input("EPSG:5778")).holds(srs::UnsupportedSystem::NotGeodetic));
}

TEST_CASE("kind gives the kind of the coordinates, which for a compound system is that of its horizontal part", "[srs][System]") {
    std::string input;
    srs::Kind expected;
    SECTION("geographic") {
        input = "EPSG:4326";
        expected = srs::Kind::Geographic;
    }
    SECTION("projected") {
        input = "EPSG:3857";
        expected = srs::Kind::Projected;
    }
    SECTION("geocentric") {
        input = "EPSG:4978";
        expected = srs::Kind::Geocentric;
    }
    SECTION("compound with a geographic horizontal part") {
        input = "EPSG:4326+3855";
        expected = srs::Kind::Geographic;
    }
    SECTION("compound with a projected horizontal part") {
        input = "EPSG:31287+5778";
        expected = srs::Kind::Projected;
    }
    const srs::System system = REQUIRE_VALUE(srs::from_user_input(input));

    CHECK(system.kind() == expected);
}

TEST_CASE("axes gives the axes in data order", "[srs][System]") {
    SECTION("geographic, whose authority order is latitude first") {
        const srs::System system = REQUIRE_VALUE(srs::from_epsg(4326));
        const auto directions = system.axes() | std::views::transform(&srs::Axis::direction) | std::ranges::to<std::vector>();
        CHECK(directions == std::vector{srs::AxisDirection::East, srs::AxisDirection::North});
    }
    SECTION("projected, whose authority order is northing first") {
        const srs::System system = REQUIRE_VALUE(srs::from_epsg(31287));
        const auto directions = system.axes() | std::views::transform(&srs::Axis::direction) | std::ranges::to<std::vector>();
        CHECK(directions == std::vector{srs::AxisDirection::East, srs::AxisDirection::North});
    }
    SECTION("compound") {
        const srs::System system = REQUIRE_VALUE(srs::from_user_input("EPSG:31287+5778"));
        const auto directions = system.axes() | std::views::transform(&srs::Axis::direction) | std::ranges::to<std::vector>();
        CHECK(directions == std::vector{srs::AxisDirection::East, srs::AxisDirection::North, srs::AxisDirection::Up});
    }
}

TEST_CASE("axes gives angles for geographic horizontal axes and lengths for the others", "[srs][System]") {
    SECTION("geographic with a height") {
        const srs::System system = REQUIRE_VALUE(srs::from_epsg(4979));
        const auto quantities = system.axes() | std::views::transform(&srs::Axis::unit) | std::views::transform(&srs::Unit::quantity) | std::ranges::to<std::vector>();
        CHECK(quantities == std::vector{srs::Quantity::Angle, srs::Quantity::Angle, srs::Quantity::Length});
    }
    SECTION("compound") {
        const srs::System system = REQUIRE_VALUE(srs::from_user_input("EPSG:31287+5778"));
        const auto quantities = system.axes() | std::views::transform(&srs::Axis::unit) | std::views::transform(&srs::Unit::quantity) | std::ranges::to<std::vector>();
        CHECK(quantities == std::vector{srs::Quantity::Length, srs::Quantity::Length, srs::Quantity::Length});
    }
}

