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
#include <optional>
#include <stop_token>

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "inputs.h"
#include "io/filesystem.h"
#include "merge.h"
#include "mesh/SimpleMesh.h"
#include "mesh/TriangleSoup.h"
#include "mesh/storage.h"
#include "octree/Id.h"
#include "octree/path_layout.h"
#include "sf/validate_index.h"
#include "stop.h"
#include "store/inputs.h"
#include "store/open.h"

namespace {

mesh::Simple sample_mesh() { return mesh::Simple({ { 0, 1, 2 } }, { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 } }); }

} // namespace

TEST_CASE("merge_datasets rejects an input with an inner node", "[sf_merger][merge]")
{
    const TempDirectory directory("merge-inner-input");
    const std::filesystem::path left_path = directory.path() / "left";
    const std::filesystem::path right_path = directory.path() / "right";
    const std::filesystem::path output_path = directory.path() / "output";
    const octree::Id root = octree::Id::root();
    {
        mesh::storage::Draft draft = REQUIRE_VALUE(mesh::storage::create(left_path, octree::path_layout::flat::layout(), ".sfmesh"));
        REQUIRE_VALUE(draft.save(root, sample_mesh()));
        REQUIRE_VALUE(draft.save(root.child(0).value(), sample_mesh()));
        REQUIRE_VALUE(mesh::storage::publish(std::move(draft)));
    }
    {
        mesh::storage::Draft draft = REQUIRE_VALUE(mesh::storage::create(right_path, octree::path_layout::flat::layout(), ".sfmesh"));
        REQUIRE_VALUE(mesh::storage::publish(std::move(draft)));
    }
    const mesh::storage::Storage left = REQUIRE_VALUE(mesh::storage::open(left_path));
    const mesh::storage::Storage right = REQUIRE_VALUE(mesh::storage::open(right_path));
    const sf_merger::inputs::Merge inputs = REQUIRE_VALUE(sf_merger::inputs::make_merge(left_path, right_path, std::nullopt));

    const auto error = REQUIRE_ERROR(merge_datasets(left, right, output_path, std::nullopt, inputs));

    CHECK(error.holds_code(sf::UnexpectedInnerNode { root }));
    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(output_path)));
}

TEST_CASE("merge_datasets links unchanged nodes and writes merged nodes", "[sf_merger][merge]")
{
    const TempDirectory directory("merge-nodes");
    const std::filesystem::path left_path = directory.path() / "left";
    const std::filesystem::path right_path = directory.path() / "right";
    const std::filesystem::path output_path = directory.path() / "output";
    const octree::Id root = octree::Id::root();

    SECTION("unchanged")
    {
        {
            mesh::storage::Draft draft = REQUIRE_VALUE(mesh::storage::create(left_path, octree::path_layout::flat::layout(), ".sfmesh"));
            REQUIRE_VALUE(draft.save(root, sample_mesh()));
            REQUIRE_VALUE(mesh::storage::publish(std::move(draft)));
        }
        {
            mesh::storage::Draft draft = REQUIRE_VALUE(mesh::storage::create(right_path, octree::path_layout::flat::layout(), ".sfmesh"));
            REQUIRE_VALUE(mesh::storage::publish(std::move(draft)));
        }
        const mesh::storage::Storage left = REQUIRE_VALUE(mesh::storage::open(left_path));
        const mesh::storage::Storage right = REQUIRE_VALUE(mesh::storage::open(right_path));
        const sf_merger::inputs::Merge inputs = REQUIRE_VALUE(sf_merger::inputs::make_merge(left_path, right_path, std::nullopt));

        REQUIRE_VALUE(merge_datasets(left, right, output_path, std::nullopt, inputs));

        const mesh::storage::Storage output = REQUIRE_VALUE(mesh::storage::open(output_path));
        REQUIRE(output.has(root));
        CHECK(REQUIRE_VALUE(io::filesystem::equivalent(left.paths(root).front(), output.paths(root).front())));
    }

    SECTION("merged")
    {
        const mesh::Simple left_mesh({ { 0, 1, 2 } }, { { -1.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 2.0, 0.0 } });
        const mesh::Simple right_mesh({ { 0, 1, 2 } }, { { 9.0, 0.0, 0.0 }, { 11.0, 0.0, 0.0 }, { 10.0, 2.0, 0.0 } });
        {
            mesh::storage::Draft draft = REQUIRE_VALUE(mesh::storage::create(left_path, octree::path_layout::flat::layout(), ".sfmesh"));
            REQUIRE_VALUE(draft.save(root, left_mesh));
            REQUIRE_VALUE(mesh::storage::publish(std::move(draft)));
        }
        {
            mesh::storage::Draft draft = REQUIRE_VALUE(mesh::storage::create(right_path, octree::path_layout::flat::layout(), ".sfmesh"));
            REQUIRE_VALUE(draft.save(root, right_mesh));
            REQUIRE_VALUE(mesh::storage::publish(std::move(draft)));
        }
        const mesh::storage::Storage left = REQUIRE_VALUE(mesh::storage::open(left_path));
        const mesh::storage::Storage right = REQUIRE_VALUE(mesh::storage::open(right_path));
        const sf_merger::inputs::Merge inputs = REQUIRE_VALUE(sf_merger::inputs::make_merge(left_path, right_path, std::nullopt));

        REQUIRE_VALUE(merge_datasets(left, right, output_path, std::nullopt, inputs));

        const mesh::storage::Storage output = REQUIRE_VALUE(mesh::storage::open(output_path));
        const mesh::Simple merged = REQUIRE_VALUE(output.load(root));
        const mesh::Simple expected(
            { { 0, 1, 2 }, { 3, 4, 5 } },
            { { -1.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 2.0, 0.0 }, { 9.0, 0.0, 0.0 }, { 11.0, 0.0, 0.0 }, { 10.0, 2.0, 0.0 } });
        CHECK(to_sorted_triangle_soup(merged) == to_sorted_triangle_soup(expected));
        CHECK_FALSE(REQUIRE_VALUE(io::filesystem::equivalent(left.paths(root).front(), output.paths(root).front())));
        CHECK_FALSE(REQUIRE_VALUE(io::filesystem::equivalent(right.paths(root).front(), output.paths(root).front())));
    }
}

TEST_CASE("merge_datasets stops on request and reuses a checkpoint", "[sf_merger][merge]")
{
    const TempDirectory directory("merge-stop-checkpoint");
    const std::filesystem::path left_path = directory.path() / "left";
    const std::filesystem::path right_path = directory.path() / "right";
    const std::filesystem::path output_path = directory.path() / "output";
    const octree::Id root = octree::Id::root();
    {
        mesh::storage::Draft draft = REQUIRE_VALUE(mesh::storage::create(left_path, octree::path_layout::flat::layout(), ".sfmesh"));
        REQUIRE_VALUE(draft.save(root, sample_mesh()));
        REQUIRE_VALUE(mesh::storage::publish(std::move(draft)));
    }
    {
        mesh::storage::Draft draft = REQUIRE_VALUE(mesh::storage::create(right_path, octree::path_layout::flat::layout(), ".sfmesh"));
        REQUIRE_VALUE(draft.save(root, sample_mesh()));
        REQUIRE_VALUE(mesh::storage::publish(std::move(draft)));
    }
    const mesh::storage::Storage left = REQUIRE_VALUE(mesh::storage::open(left_path));
    const mesh::storage::Storage right = REQUIRE_VALUE(mesh::storage::open(right_path));
    const sf_merger::inputs::Merge inputs = REQUIRE_VALUE(sf_merger::inputs::make_merge(left_path, right_path, std::nullopt));

    SECTION("stop")
    {
        std::stop_source stop;
        stop.request_stop();

        const auto error = REQUIRE_ERROR(merge_datasets(left, right, output_path, std::nullopt, inputs, std::nullopt, stop.get_token()));

        CHECK(error.holds<Stopped>());
        CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(output_path)));
        const std::filesystem::path draft_path = directory.path() / "output.part";
        CHECK(REQUIRE_VALUE(io::filesystem::exists(draft_path / store::inputs::file_name)));
        REQUIRE_VALUE(mesh::storage::open(draft_path, store::Accept::Draft));
    }

    SECTION("checkpoint")
    {
        const std::filesystem::path first_path = directory.path() / "first";
        REQUIRE_VALUE(merge_datasets(left, right, first_path, std::nullopt, inputs));
        // Turn the published output into what an interrupted merge leaves behind.
        const std::filesystem::path checkpoint_path = directory.path() / "checkpoint.part";
        REQUIRE_VALUE(io::filesystem::rename(first_path, checkpoint_path));
        REQUIRE_VALUE(store::inputs::write<sf_merger::inputs::Schema>(checkpoint_path, inputs));

        REQUIRE_VALUE(merge_datasets(left, right, output_path, std::nullopt, inputs, checkpoint_path));

        const mesh::storage::Storage checkpoint = REQUIRE_VALUE(mesh::storage::open(checkpoint_path, store::Accept::Draft));
        const mesh::storage::Storage output = REQUIRE_VALUE(mesh::storage::open(output_path));
        CHECK(REQUIRE_VALUE(io::filesystem::equivalent(checkpoint.paths(root).front(), output.paths(root).front())));
    }
}
