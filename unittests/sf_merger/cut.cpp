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

#include <catch2/catch_test_macros.hpp>
#include <glm/glm.hpp>

#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "cut.h"
#include "io/filesystem.h"
#include "mask.h"
#include "mesh/SimpleMesh.h"
#include "mesh/bounds.h"
#include "mesh/geometry.h"
#include "mesh/primitives.h"
#include "mesh/storage.h"
#include "octree/Id.h"
#include "octree/path_layout.h"
#include "sf/validate_index.h"

namespace {

mesh::Simple sample_mesh() { return mesh::Simple({ { 0, 1, 2 } }, { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 } }); }

} // namespace

TEST_CASE("cut_dataset rejects an input with an inner node", "[sf_merger][cut]")
{
    const TempDirectory directory("cut-inner-input");
    const std::filesystem::path input_path = directory.path() / "input";
    const std::filesystem::path output_path = directory.path() / "output";
    const octree::Id root = octree::Id::root();
    {
        mesh::storage::Draft draft = REQUIRE_VALUE(mesh::storage::create(input_path, octree::path_layout::flat::layout(), ".sfmesh"));
        REQUIRE_VALUE(draft.save(root, sample_mesh()));
        REQUIRE_VALUE(draft.save(root.child(0).value(), sample_mesh()));
        REQUIRE_VALUE(mesh::storage::publish(std::move(draft)));
    }
    const mesh::storage::Storage input = REQUIRE_VALUE(mesh::storage::open(input_path));

    const auto error = REQUIRE_ERROR(cut_dataset(input, MeshMask {}, output_path, false));

    CHECK(error.holds_code(sf::UnexpectedInnerNode { root }));
    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(output_path)));
}

TEST_CASE("cut_dataset links, clips or drops each leaf", "[sf_merger][cut]")
{
    const TempDirectory directory("cut-leaves");
    const std::filesystem::path input_path = directory.path() / "input";
    const std::filesystem::path output_path = directory.path() / "output";
    const octree::Id root = octree::Id::root();
    const mesh::Simple triangle({ { 0, 1, 2 } }, { { -1.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 2.0, 0.0 } });
    {
        mesh::storage::Draft draft = REQUIRE_VALUE(mesh::storage::create(input_path, octree::path_layout::flat::layout(), ".sfmesh"));
        REQUIRE_VALUE(draft.save(root, triangle));
        REQUIRE_VALUE(mesh::storage::publish(std::move(draft)));
    }
    const mesh::storage::Storage input = REQUIRE_VALUE(mesh::storage::open(input_path));

    SECTION("unchanged")
    {
        REQUIRE_VALUE(cut_dataset(input, MeshMask {}, output_path, false));

        const mesh::storage::Storage output = REQUIRE_VALUE(mesh::storage::open(output_path));
        REQUIRE(output.has(root));
        CHECK(REQUIRE_VALUE(io::filesystem::equivalent(input.paths(root).front(), output.paths(root).front())));
    }

    SECTION("clipped")
    {
        const MeshMask mask { .components = { mesh::make_box(radix::geometry::Aabb<3, double>{ { 0.0, -1.0, -1.0 }, { 2.0, 3.0, 1.0 } }) } };

        REQUIRE_VALUE(cut_dataset(input, mask, output_path, true));

        const mesh::storage::Storage output = REQUIRE_VALUE(mesh::storage::open(output_path));
        const mesh::Simple clipped = REQUIRE_VALUE(output.load(root));
        // Any triangulation of (0, 0, 0), (1, 0, 0), (0, 2, 0) is correct, and the clip may add vertices on its edges.
        const std::vector<radix::geometry::Aabb3d> clipped_bounds { mesh::calculate_bounds(clipped) };
        CHECK_THAT(clipped_bounds, UnorderedWithinEpsilon(std::vector<radix::geometry::Aabb3d> { { { 0.0, 0.0, 0.0 }, { 1.0, 2.0, 0.0 } } }));
        CHECK_THAT(mesh::compute_surface_area(clipped), WithinEpsilon(1.0));
        CHECK_FALSE(REQUIRE_VALUE(io::filesystem::equivalent(input.paths(root).front(), output.paths(root).front())));
    }

    SECTION("dropped")
    {
        const MeshMask mask { .components = { mesh::make_box(radix::geometry::Aabb<3, double>{ { 5.0, -1.0, -1.0 }, { 7.0, 3.0, 1.0 } }) } };

        REQUIRE_VALUE(cut_dataset(input, mask, output_path, true));

        const mesh::storage::Storage output = REQUIRE_VALUE(mesh::storage::open(output_path));
        CHECK_FALSE(output.has(root));
    }
}
