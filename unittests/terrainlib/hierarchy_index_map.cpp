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

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hierarchy/IndexMap.h"
#include "hierarchy/NodeStatus.h"
#include "hierarchy/Traits.h"
#include "octree/StoreTraits.h"
#include "raster_store/StoreTraits.h"

static_assert(hierarchy::Traits<octree::HierarchyTraits>);
static_assert(hierarchy::Traits<raster_store::HierarchyTraits>);

namespace {

struct OctreeKeys {
    using Keys = octree::KeyTraits;
    using Tree = octree::HierarchyTraits;
};

struct RasterKeys {
    using Keys = raster_store::KeyTraits;
    using Tree = raster_store::HierarchyTraits;
};

} // namespace

TEMPLATE_TEST_CASE("IndexMap transitions", "[hierarchy][IndexMap]", OctreeKeys, RasterKeys)
{
    using Tree = typename TestType::Tree;

    hierarchy::IndexMap<typename TestType::Keys, Tree> index;
    const auto root = Tree::root();
    const auto children = Tree::children(root).value();
    const auto first = children[0];
    const auto second = children[1];

    REQUIRE(index.add(first));
    CHECK(index.is(hierarchy::NodeStatus::Virtual, root));
    CHECK(index.is(hierarchy::NodeStatus::Leaf, first));

    REQUIRE(index.add(root));
    CHECK(index.is(hierarchy::NodeStatus::Inner, root));
    CHECK_FALSE(index.add(root));

    REQUIRE(index.add(second));
    REQUIRE(index.remove(first));
    CHECK(index.is(hierarchy::NodeStatus::Inner, root));
    REQUIRE(index.remove(second));
    CHECK(index.is(hierarchy::NodeStatus::Leaf, root));
    REQUIRE(index.remove(root));
    CHECK(index.empty());
}
