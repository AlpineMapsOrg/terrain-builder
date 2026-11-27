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

#include <vector>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hierarchy/IndexMap.h"
#include "hierarchy/NodeStatus.h"
#include "hierarchy/traverse.h"
#include "octree/StoreTraits.h"
#include "raster_store/StoreTraits.h"

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

TEMPLATE_TEST_CASE("traverse preserves the trees' child order", "[hierarchy][traverse]", OctreeKeys, RasterKeys)
{
    using Tree = typename TestType::Tree;
    using Key = typename TestType::Keys::Key;

    hierarchy::IndexMap<typename TestType::Keys, Tree> index;
    const Key root = Tree::root();
    const auto children = Tree::children(root).value();
    const Key first = children[0];
    const Key second = children[1];
    const Key grandchild = Tree::children(first).value()[0];
    REQUIRE(index.add(root));
    REQUIRE(index.add(first));
    REQUIRE(index.add(second));
    REQUIRE(index.add(grandchild));

    std::vector<Key> depth_first;
    hierarchy::traverse(index, [&](const Key& key, const hierarchy::NodeStatus) { depth_first.push_back(key); });
    CHECK(depth_first == std::vector<Key> { root, first, grandchild, second });

    std::vector<Key> breadth_first;
    hierarchy::traverse(
        index,
        [&](const Key& key, const hierarchy::NodeStatus) { breadth_first.push_back(key); },
        hierarchy::AlwaysRefine {},
        root,
        hierarchy::TraversalOrder::BreadthFirst);
    CHECK(breadth_first == std::vector<Key> { root, first, second, grandchild });

    std::vector<Key> explicit_root;
    hierarchy::traverse(index, [&](const Key& key, const hierarchy::NodeStatus) { explicit_root.push_back(key); }, hierarchy::AlwaysRefine {}, first);
    CHECK(explicit_root == std::vector<Key> { first, grandchild });
}

TEMPLATE_TEST_CASE("traverse descends only into nodes that refine accepts", "[hierarchy][traverse]", OctreeKeys, RasterKeys)
{
    using Tree = typename TestType::Tree;
    using Key = typename TestType::Keys::Key;

    hierarchy::IndexMap<typename TestType::Keys, Tree> index;
    const Key root = Tree::root();
    const auto children = Tree::children(root).value();
    const Key first = children[0];
    const Key second = children[1];
    const Key grandchild = Tree::children(first).value()[0];
    REQUIRE(index.add(root));
    REQUIRE(index.add(first));
    REQUIRE(index.add(second));
    REQUIRE(index.add(grandchild));

    std::vector<Key> visited;
    hierarchy::traverse(index, [&](const Key& key, const hierarchy::NodeStatus) { visited.push_back(key); }, [&](const Key& key) { return key == root; });
    CHECK(visited == std::vector<Key> { root, first, second });
}

TEMPLATE_TEST_CASE("traverse visits nothing from a root the index does not list", "[hierarchy][traverse]", OctreeKeys, RasterKeys)
{
    using Tree = typename TestType::Tree;
    using Key = typename TestType::Keys::Key;

    hierarchy::IndexMap<typename TestType::Keys, Tree> index;
    const Key root = Tree::root();
    const auto children = Tree::children(root).value();
    const Key first = children[0];
    const Key second = children[1];
    const Key grandchild = Tree::children(first).value()[0];
    REQUIRE(index.add(root));
    REQUIRE(index.add(first));
    REQUIRE(index.add(second));
    REQUIRE(index.add(grandchild));
    const Key absent = children[2];

    std::vector<Key> visited;
    hierarchy::traverse(index, [&](const Key& key, const hierarchy::NodeStatus) { visited.push_back(key); }, hierarchy::AlwaysRefine {}, absent);
    CHECK(visited == std::vector<Key> {});
}
