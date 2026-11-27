#include "../catch2_helpers.h"
#include "../result_helpers.h"

#include <string>

#include "octree/Id.h"

#include "json/octree.h"
#include "json/serialize.h"

TEST_CASE("read_from_text reads an id from its level and index", "[json][octree]") {
    CHECK(REQUIRE_VALUE(json::read_from_text<octree::Id>("[2, 5]")) == octree::Id(2, octree::Id::Index{5}));
}

TEST_CASE("write_to_text writes an id that reads back equal", "[json][octree]") {
    const std::string text = REQUIRE_VALUE(json::write_to_text(octree::Id(2, octree::Id::Index{5})));
    CHECK(REQUIRE_VALUE(json::read_from_text<octree::Id>(text)) == octree::Id(2, octree::Id::Index{5}));
}

TEST_CASE("read_from_text rejects an index outside its level", "[json][octree]") {
    // Level 1 holds the indices 0 to 7.
    REQUIRE_ERROR(json::read_from_text<octree::Id>("[1, 8]"));
}

TEST_CASE("read_from_text rejects a level too deep", "[json][octree]") {
    REQUIRE_ERROR(json::read_from_text<octree::Id>("[22, 0]"));
}

TEST_CASE("read_from_text rejects a wrong number of entries", "[json][octree]") {
    REQUIRE_ERROR(json::read_from_text<octree::Id>("[2]"));
    REQUIRE_ERROR(json::read_from_text<octree::Id>("[2, 5, 1]"));
}
