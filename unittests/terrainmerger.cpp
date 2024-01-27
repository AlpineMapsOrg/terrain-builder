/*****************************************************************************
 * Alpine Terrain Builder
 * Copyright (C) 2022 Adam Celarek <last name at cg tuwien ac at>
 * Copyright (C) 2022 alpinemaps.org
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

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include "catch2_helpers.h"
#include "merge.h"

TEST_CASE("terrainmerger") {
    SECTION("two tris") {
        TerrainMesh mesh1;
        mesh1.positions.push_back(glm::dvec3(0, 0, 0));
        mesh1.positions.push_back(glm::dvec3(1, 1, 0));
        mesh1.positions.push_back(glm::dvec3(1, 0, 0));
        mesh1.triangles.push_back(glm::uvec3(0, 1, 2));

        TerrainMesh mesh2;
        mesh2.positions.push_back(glm::dvec3(1, 0, 0));
        mesh2.positions.push_back(glm::dvec3(1, 1, 0));
        mesh2.positions.push_back(glm::dvec3(0, 1, 0));
        mesh2.triangles.push_back(glm::uvec3(0, 1, 2));

        std::array<TerrainMesh, 2> meshes = { std::move(mesh1), std::move(mesh2) };

        TerrainMesh actual = merge::merge_by_distance(meshes, 0.1);

        TerrainMesh expected;
        expected.positions.push_back(glm::dvec3(0, 0, 0));
        expected.positions.push_back(glm::dvec3(1, 1, 0));
        expected.positions.push_back(glm::dvec3(1, 0, 0));
        expected.positions.push_back(glm::dvec3(0, 1, 0));
        expected.triangles.push_back(glm::uvec3(0, 1, 2));
        expected.triangles.push_back(glm::uvec3(2, 1, 3));

        REQUIRE(expected.positions == actual.positions);
        REQUIRE(expected.uvs == actual.uvs);
        REQUIRE(expected.triangles == actual.triangles);
    }

    SECTION("two tris with uvs") {
        TerrainMesh mesh1;
        mesh1.positions.push_back(glm::dvec3(0, 0, 0));
        mesh1.positions.push_back(glm::dvec3(1, 1, 0));
        mesh1.positions.push_back(glm::dvec3(1, 0, 0));
        mesh1.uvs.push_back(glm::dvec2(0, 0));
        mesh1.uvs.push_back(glm::dvec2(1, 1));
        mesh1.uvs.push_back(glm::dvec2(1, 0));
        mesh1.triangles.push_back(glm::uvec3(0, 1, 2));

        TerrainMesh mesh2;
        mesh2.positions.push_back(glm::dvec3(1, 0, 0));
        mesh2.positions.push_back(glm::dvec3(1, 1, 0));
        mesh2.positions.push_back(glm::dvec3(0, 1, 0));
        mesh2.uvs.push_back(glm::dvec2(1, 0));
        mesh2.uvs.push_back(glm::dvec2(1, 1));
        mesh2.uvs.push_back(glm::dvec2(0, 1));
        mesh2.triangles.push_back(glm::uvec3(0, 1, 2));

        std::array<TerrainMesh, 2> meshes = {std::move(mesh1), std::move(mesh2)};

        TerrainMesh actual = merge::merge_by_distance(meshes, 0.1);

        TerrainMesh expected;
        expected.positions.push_back(glm::dvec3(0, 0, 0));
        expected.positions.push_back(glm::dvec3(1, 1, 0));
        expected.positions.push_back(glm::dvec3(1, 0, 0));
        expected.positions.push_back(glm::dvec3(0, 1, 0));
        expected.uvs.push_back(glm::dvec2(0, 0));
        expected.uvs.push_back(glm::dvec2(1, 1));
        expected.uvs.push_back(glm::dvec2(1, 0));
        expected.uvs.push_back(glm::dvec2(0, 1));
        expected.triangles.push_back(glm::uvec3(0, 1, 2));
        expected.triangles.push_back(glm::uvec3(2, 1, 3));

        REQUIRE(expected.positions == actual.positions);
        REQUIRE(expected.uvs == actual.uvs);
        REQUIRE(expected.triangles == actual.triangles);
    }

    SECTION("fine mesh") {
        TerrainMesh mesh1;
        mesh1.positions.push_back(glm::dvec3(-1, -1, 0));
        mesh1.positions.push_back(glm::dvec3(1, -1, 0));
        mesh1.positions.push_back(glm::dvec3(-1, 1, 0));
        mesh1.positions.push_back(glm::dvec3(1, 1, 0));
        mesh1.triangles.push_back(glm::uvec3(0, 2, 1));
        mesh1.triangles.push_back(glm::uvec3(1, 2, 3));

        TerrainMesh mesh2;
        mesh2.positions.push_back(glm::dvec3(0.9, 0.9, 0));
        mesh2.positions.push_back(glm::dvec3(1.1, 0.9, 0));
        mesh2.positions.push_back(glm::dvec3(-1, 0.9, 0));
        mesh2.positions.push_back(glm::dvec3(0.9, 0.9, 0));
        mesh2.triangles.push_back(glm::uvec3(0, 2, 1));
        mesh2.triangles.push_back(glm::uvec3(1, 2, 3));

        std::array<TerrainMesh, 2> meshes = {std::move(mesh1), std::move(mesh2)};

        TerrainMesh actual = merge::merge_by_distance(meshes, 0.1);

        TerrainMesh expected;
        expected.positions.push_back(glm::dvec3(-1, -1, 0));
        expected.positions.push_back(glm::dvec3(1, -1, 0));
        expected.positions.push_back(glm::dvec3(-1, 1, 0));
        expected.positions.push_back(glm::dvec3(1, 1, 0));
        expected.triangles.push_back(glm::uvec3(0, 2, 1));
        expected.triangles.push_back(glm::uvec3(1, 2, 3));

        REQUIRE(expected.positions == actual.positions);
        REQUIRE(expected.uvs == actual.uvs);
        REQUIRE(expected.triangles == actual.triangles);
    }
}
