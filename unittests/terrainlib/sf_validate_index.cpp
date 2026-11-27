/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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

#include "../result_helpers.h"

#include <catch2/catch_test_macros.hpp>

#include <optional>

#include "octree/Id.h"
#include "octree/storage/Index.h"
#include "sf/validate_index.h"

TEST_CASE("validate_index accepts leaf and virtual nodes", "[sf][validate_index]") {
    octree::storage::Index index;
    const octree::Id leaf = octree::Id::root().child(2).value().child(5).value();
    REQUIRE(index.add(leaf, std::nullopt));

    REQUIRE_VALUE(sf::validate_index(index));
}

TEST_CASE("validate_index fails for an inner node", "[sf][validate_index]") {
    octree::storage::Index index;
    const octree::Id root = octree::Id::root();
    REQUIRE(index.add(root, std::nullopt));
    REQUIRE(index.add(root.child(3).value(), std::nullopt));

    const auto error = REQUIRE_ERROR(sf::validate_index(index));

    CHECK(error.holds_code(sf::UnexpectedInnerNode { root }));
}
