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
#include <ranges>
#include <stop_token>

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "build.h"
#include "inputs.h"
#include "io/filesystem.h"
#include "mesh/SimpleMesh.h"
#include "mesh/storage.h"
#include "octree/Id.h"
#include "octree/path_layout.h"
#include "octree/storage/Index.h"
#include "octree/storage/index_format.h"
#include "range/RangeFull.h"
#include "sf/validate_index.h"
#include "stop.h"
#include "storage.h"
#include "store/Checkpoint.h"
#include "store/inputs.h"

namespace {

mesh::Simple sample_mesh()
{
    return mesh::Simple({ { 0, 1, 2 } }, { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 } });
}

} // namespace

TEST_CASE("DAG builder rejects invalid SF topology before processing", "[dag_builder][build]")
{
    const TempDirectory directory("dag-invalid-topology");
    const std::filesystem::path input_path = directory.path() / "input";
    const octree::Id root = octree::Id::root();
    {
        mesh::storage::Draft input_draft = REQUIRE_VALUE(mesh::storage::create(input_path, octree::path_layout::flat::layout(), ".sfmesh"));
        REQUIRE_VALUE(input_draft.save(root, sample_mesh()));
        REQUIRE_VALUE(input_draft.save(root.child(0).value(), sample_mesh()));
        REQUIRE(input_draft.index().status(root) == hierarchy::NodeStatus::Inner);
        REQUIRE_VALUE(mesh::storage::publish(std::move(input_draft)));
    }
    const mesh::storage::Storage input = REQUIRE_VALUE(mesh::storage::open(input_path));
    dag::storage::Draft output = REQUIRE_VALUE(dag::storage::create(directory.path() / "output", octree::storage::default_layout()));
    const dag::BuildOptions options {
        .clusters_per_partition = 1,
        .target_ratio = 1.0f,
        .relative_target_error = std::nullopt,
    };

    const auto error = REQUIRE_ERROR(dag::build_full(input, output, options));
    CHECK(error.holds_code(dag::BuildIoError::InvalidIndex));
    REQUIRE(error.source() != nullptr);
    CHECK(error.source()->holds_code(sf::UnexpectedInnerNode { root }));
    const octree::storage::Index output_index = output.index();
    CHECK(std::ranges::empty(output_index.keys()));
}

TEST_CASE("DAG builds stop on request and reuse checkpointed nodes", "[dag_builder][build][inputs]")
{
    const TempDirectory directory("dag-stop-checkpoint");
    const std::filesystem::path input_path = directory.path() / "input";
    {
        mesh::storage::Draft input_draft = REQUIRE_VALUE(mesh::storage::create(input_path, octree::path_layout::flat::layout(), ".sfmesh"));
        REQUIRE_VALUE(input_draft.save(octree::Id::root(), sample_mesh()));
        REQUIRE_VALUE(mesh::storage::publish(std::move(input_draft)));
    }
    const mesh::storage::Storage input = REQUIRE_VALUE(mesh::storage::open(input_path));
    const dag::BuildOptions options {
        .clusters_per_partition = 1,
        .target_ratio = 1.0f,
        .relative_target_error = std::nullopt,
    };

    SECTION("stop")
    {
        dag::storage::Draft output = REQUIRE_VALUE(dag::storage::create(directory.path() / "stopped", octree::storage::default_layout()));
        std::stop_source stop_source;
        stop_source.request_stop();

        const auto error = REQUIRE_ERROR(dag::build_levels(input, output, options, RangeFull {}, {}, nullptr, stop_source.get_token()));
        CHECK(error.holds<Stopped>());
        const octree::storage::Index output_index = output.index();
        CHECK(std::ranges::empty(output_index.keys()));
    }

    SECTION("checkpoint")
    {
        const dag::inputs::Build inputs = REQUIRE_VALUE(dag::inputs::make_build(input_path, options, RangeFull {}));
        dag::storage::Draft first = REQUIRE_VALUE(dag::storage::create(directory.path() / "first", octree::storage::default_layout()));
        REQUIRE_VALUE(dag::build_levels(input, first, options, RangeFull {}));
        REQUIRE_VALUE(first.save_index());
        REQUIRE_VALUE(store::inputs::write<dag::inputs::Schema>(first.base_path(), inputs));
        const octree::storage::Index first_index = first.index();
        REQUIRE_FALSE(std::ranges::empty(first_index.keys()));

        const std::filesystem::path output_path = directory.path() / "second";
        dag::storage::Draft output = REQUIRE_VALUE(dag::storage::create(output_path, octree::storage::default_layout()));
        auto open_result = store::open_for_reuse<dag::inputs::Schema>(first.base_path(), output_path, inputs, dag::storage::open);
        const auto checkpoint = REQUIRE_VALUE(std::move(open_result));
        REQUIRE_VALUE(dag::build_levels(input, output, options, RangeFull {}, checkpoint));

        const octree::storage::Index output_index = output.index();
        CHECK(std::ranges::distance(output_index.keys()) == std::ranges::distance(first_index.keys()));
        for (const octree::Id id : first_index.keys()) {
            REQUIRE(REQUIRE_VALUE(io::filesystem::equivalent(first.paths(id).front(), output.paths(id).front())));
        }
    }
}
