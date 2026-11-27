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

#include <cstdint>
#include <filesystem>
#include <optional>
#include <ranges>
#include <stop_token>

#include <catch2/catch_test_macros.hpp>
#include <glm/glm.hpp>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "build.h"
#include "common_error_codes.h"
#include "gis/srs/well_known.h"
#include "inputs.h"
#include "io/filesystem.h"
#include "mesh/SimpleMesh.h"
#include "mesh/storage.h"
#include "octree/Id.h"
#include "stop.h"
#include "store/inputs.h"
#include "store/open.h"
#include "test_raster.h"

namespace {

// The inputs of a batch build of the dataset at `dataset_path` at `target_level`.
sf::inputs::Build batch_inputs(const std::filesystem::path& dataset_path, const uint32_t target_level) {
    sf::inputs::Build inputs;
    inputs.dataset = dataset_path.string();
    inputs.mesh_srs = "EPSG:4978";
    inputs.target_level = target_level;
    inputs.format = ".sfmesh";
    return inputs;
}

} // namespace

TEST_CASE("batch builds datasets crossing the prime meridian", "[build]") {
    const TempDirectory directory("sf-coverage-batch");
    // Crosses the plane between two level-one nodes, so traversal starts at the root.
    const std::filesystem::path dataset_path = write_raster(directory.path(), gis::srs::wgs84(), {{-0.02, 51.47}, {0.02, 51.49}}, {400, 200});
    const std::filesystem::path output = directory.path() / "output";

    SECTION("target levels below the minimum are rejected") {
        const auto error = REQUIRE_ERROR(sf::build_and_publish_all_patches(batch_inputs(dataset_path, 1), output));
        CHECK(error.holds<Unsupported>());
        CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(output)));
        CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(directory.path() / "output.part")));
    }

    SECTION("nodes on both sides are built") {
        REQUIRE_VALUE(sf::build_and_publish_all_patches(batch_inputs(dataset_path, 12), output));
        CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(directory.path() / "output.part")));
        const mesh::storage::Storage storage = REQUIRE_VALUE(mesh::storage::open(output));
        size_t east = 0;
        size_t west = 0;
        for (const octree::Id node : storage.index().keys()) {
            const SimpleMesh mesh = REQUIRE_VALUE(storage.load(node));
            for (const glm::dvec3& position : mesh.positions) {
                (position.y > 0 ? east : west)++;
            }
        }
        CHECK(east > 1000);
        CHECK(west > 1000);
    }
}

TEST_CASE("batch builds stop on request and reuse checkpointed nodes", "[build]") {
    const TempDirectory directory("sf-batch-checkpoint");
    const std::filesystem::path dataset_path = write_raster(directory.path(), gis::srs::wgs84(), {{-0.02, 51.47}, {0.02, 51.49}}, {400, 200});
    const sf::inputs::Build inputs = batch_inputs(dataset_path, 12);
    const std::filesystem::path output = directory.path() / "output";

    SECTION("a stop request retains the draft") {
        // The stop is requested before the build starts, so no node is built and the retained draft has an empty index.
        std::stop_source stop_source;
        stop_source.request_stop();
        const auto error = REQUIRE_ERROR(sf::build_and_publish_all_patches(inputs, output, std::nullopt, stop_source.get_token()));
        CHECK(error.holds<Stopped>());
        CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(output)));
        const std::filesystem::path draft = directory.path() / "output.part";
        CHECK(REQUIRE_VALUE(io::filesystem::exists(draft / store::inputs::file_name)));
        REQUIRE_VALUE(mesh::storage::open(draft, store::Accept::Draft));
    }

    SECTION("checkpointed nodes are linked") {
        const std::filesystem::path first = directory.path() / "first";
        REQUIRE_VALUE(sf::build_and_publish_all_patches(inputs, first));
        // Turn the published output into what an interrupted build leaves behind.
        const std::filesystem::path checkpoint = directory.path() / "checkpoint.part";
        REQUIRE_VALUE(io::filesystem::rename(first, checkpoint));
        REQUIRE_VALUE(store::inputs::write<sf::inputs::Schema>(checkpoint, inputs));

        REQUIRE_VALUE(sf::build_and_publish_all_patches(inputs, output, checkpoint));
        const mesh::storage::Storage checkpointed = REQUIRE_VALUE(mesh::storage::open(checkpoint, store::Accept::Draft));
        const mesh::storage::Storage built = REQUIRE_VALUE(mesh::storage::open(output));
        CHECK(built.index().map().size() == checkpointed.index().map().size());
        size_t linked = 0;
        for (const octree::Id id : checkpointed.index().keys()) {
            REQUIRE(REQUIRE_VALUE(io::filesystem::equivalent(checkpointed.paths(id).front(), built.paths(id).front())));
            linked++;
        }
        CHECK(linked > 0);
    }
}
