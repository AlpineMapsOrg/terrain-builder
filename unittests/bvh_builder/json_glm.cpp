#include "../catch2_helpers.h"
#include "../result_helpers.h"

#include <string>

#include <glm/glm.hpp>

#include "json/glm.h"
#include "json/serialize.h"

TEST_CASE("read_from_text reads a vector from an array of its components", "[json][glm]") {
    SECTION("2d") {
        CHECK(REQUIRE_VALUE(json::read_from_text<glm::dvec2>("[1.5, -2]")) == glm::dvec2(1.5, -2.0));
    }
    SECTION("3d") {
        CHECK(REQUIRE_VALUE(json::read_from_text<glm::dvec3>("[1.5, -2, 0.1]")) == glm::dvec3(1.5, -2.0, 0.1));
    }
}

TEST_CASE("write_to_text writes a vector that reads back equal", "[json][glm]") {
    SECTION("2d") {
        const std::string text = REQUIRE_VALUE(json::write_to_text(glm::dvec2(1.5, -2.0)));
        CHECK(REQUIRE_VALUE(json::read_from_text<glm::dvec2>(text)) == glm::dvec2(1.5, -2.0));
    }
    SECTION("3d") {
        const std::string text = REQUIRE_VALUE(json::write_to_text(glm::dvec3(1.5, -2.0, 0.1)));
        CHECK(REQUIRE_VALUE(json::read_from_text<glm::dvec3>(text)) == glm::dvec3(1.5, -2.0, 0.1));
    }
}

TEST_CASE("read_from_text rejects an array with the wrong number of components", "[json][glm]") {
    SECTION("2d") {
        REQUIRE_ERROR(json::read_from_text<glm::dvec2>("[1.5]"));
        REQUIRE_ERROR(json::read_from_text<glm::dvec2>("[1.5, -2, 0.1]"));
    }
    SECTION("3d") {
        REQUIRE_ERROR(json::read_from_text<glm::dvec3>("[1.5, -2]"));
        REQUIRE_ERROR(json::read_from_text<glm::dvec3>("[1.5, -2, 0.1, 4]"));
    }
}
