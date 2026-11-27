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

#include <cstdint>
#include <vector>

#include "../catch2_helpers.h"
#include "../result_helpers.h"

#include "cli_support/octree_id.h"
#include "octree/Id.h"

TEST_CASE("parse_octree_id makes the node the values name", "[cli_support][octree_id]") {
    SECTION("level and index") {
        CHECK(REQUIRE_VALUE(cli_support::parse_octree_id(std::vector<uint64_t>{3, 5})) == octree::Id(3, 5));
    }
    SECTION("level and coordinates") {
        CHECK(REQUIRE_VALUE(cli_support::parse_octree_id(std::vector<uint64_t>{2, 1, 2, 3})) == octree::Id(2, 1, 2, 3));
    }
}

TEST_CASE("parse_octree_id fails for values that name no node", "[cli_support][octree_id]") {
    SECTION("level and index") {
        REQUIRE_ERROR(cli_support::parse_octree_id(std::vector<uint64_t>{22, 0}));
        REQUIRE_ERROR(cli_support::parse_octree_id(std::vector<uint64_t>{257, 0}));
    }
    SECTION("level and coordinates") {
        REQUIRE_ERROR(cli_support::parse_octree_id(std::vector<uint64_t>{1, 2, 0, 0}));
        REQUIRE_ERROR(cli_support::parse_octree_id(std::vector<uint64_t>{257, 0, 0, 0}));
        REQUIRE_ERROR(cli_support::parse_octree_id(std::vector<uint64_t>{21, uint64_t{1} << 32, 0, 0}));
    }
}

TEST_CASE("parse_octree_id fails for a number of values other than two or four", "[cli_support][octree_id]") {
    REQUIRE_ERROR(cli_support::parse_octree_id(std::vector<uint64_t>{}));
    REQUIRE_ERROR(cli_support::parse_octree_id(std::vector<uint64_t>{3, 1, 2}));
    REQUIRE_ERROR(cli_support::parse_octree_id(std::vector<uint64_t>{3, 1, 2, 3, 4}));
}
