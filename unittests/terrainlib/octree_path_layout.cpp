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

#include <filesystem>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "mesh/codec/SfMesh.h"
#include "octree/Id.h"
#include "octree/path_layout.h"
#include "store/path_layout.h"

TEST_CASE("extensionless octree layouts preserve physical paths", "[octree][path_layout]")
{
    const std::filesystem::path dataset = "dataset";
    mesh::codec::SfMesh sfmesh;

    SECTION("flat")
    {
        const store::path_layout::Resolver layout(dataset / "sf-flat", octree::path_layout::flat::layout());
        const std::filesystem::path node_path = layout.locate_node(octree::Id::root());
        CHECK(node_path == dataset / "sf-flat/0-0");
        CHECK(sfmesh.paths(node_path) == std::vector { dataset / "sf-flat/0-0.sfmesh" });
        CHECK(layout.identify_node(node_path) == octree::Id::root());
    }

    SECTION("level and coordinate directories")
    {
        const store::path_layout::Resolver layout(dataset / "sf-coordinates", octree::path_layout::level_and_coordinate_directories::layout());
        const octree::Id child = octree::Id::root().child(2).value();
        const octree::Id deep = octree::Id::root().child(5).value().child(7).value();
        CHECK(sfmesh.paths(layout.locate_node(child)) == std::vector { dataset / "sf-coordinates/1/0/1/0.sfmesh" });
        CHECK(sfmesh.paths(layout.locate_node(deep)) == std::vector { dataset / "sf-coordinates/2/3/1/3.sfmesh" });
        CHECK(layout.identify_node(layout.locate_node(child)) == child);
        CHECK(layout.identify_node(layout.locate_node(deep)) == deep);
    }
}

TEST_CASE("octree layout lookup is explicit", "[octree][path_layout]")
{
    REQUIRE(octree::path_layout::resolve_layout("flat").has_value());
    CHECK(octree::path_layout::resolve_layout("flat")->id == "flat");
    REQUIRE(octree::path_layout::resolve_layout("level_and_coordinate_directories").has_value());
    CHECK(octree::path_layout::resolve_layout("level_and_coordinate_directories")->id == "level_and_coordinate_directories");
    CHECK_FALSE(octree::path_layout::resolve_layout("unknown").has_value());
    CHECK(octree::path_layout::all().size() == 2);
}

TEST_CASE("extensionless octree layouts validate complete paths", "[octree][path_layout]")
{
    const auto flat = octree::path_layout::flat::layout();
    CHECK_FALSE(flat.identify_node("1-2.sfmesh").has_value());
    CHECK_FALSE(flat.identify_node("nested/1-2").has_value());
    CHECK_FALSE(flat.identify_node("1-2-extra").has_value());

    const auto coordinates = octree::path_layout::level_and_coordinate_directories::layout();
    CHECK_FALSE(coordinates.identify_node("1/0/0/0.sfmesh").has_value());
    CHECK_FALSE(coordinates.identify_node("1/0/0").has_value());
    CHECK_FALSE(coordinates.identify_node("1/0/0/0/extra").has_value());
    CHECK_FALSE(coordinates.identify_node("22/0/0/0").has_value());
}

TEST_CASE("octree layouts round-trip boundary IDs", "[octree][path_layout]")
{
    const std::vector ids {
        octree::Id::root(),
        octree::Id(octree::Id::max_level(), octree::Id::Index { 0 }),
        octree::Id(octree::Id::max_level(), octree::Id::max_index_on_level(octree::Id::max_level())),
    };

    const auto flat = octree::path_layout::flat::layout();
    const auto coordinates = octree::path_layout::level_and_coordinate_directories::layout();
    for (const octree::Id id : ids) {
        const auto flat_path = flat.locate_node(id);
        CHECK(flat.identify_node(flat_path) == id);

        const auto coordinate_path = coordinates.locate_node(id);
        CHECK(coordinates.identify_node(coordinate_path) == id);
    }
}
