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

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "dag_node.h"
#include "io/filesystem.h"
#include "octree/Id.h"
#include "octree/storage/index_format.h"
#include "octree/storage/metadata.h"
#include "storage.h"

namespace {

dag::ClusterBatch sample_batch() {
    dag::ClusterBatch batch;
    batch.metadata.group_assignment = { 0 };
    batch.metadata.triangle_counts = { 1 };
    batch.metadata.groups = { {
        .children = { { octree::Id::root().child(4).value(), 3 } },
        .error = 12.5,
        .bounds = { { 0.5, 1.5, 2.5 }, { 3.5, 4.5, 5.5 } },
    } };
    batch.clustering.positions = {
        { 0.0, 0.0, 0.0 },
        { 1.0, 0.0, 0.0 },
        { 0.0, 1.0, 0.0 },
    };
    batch.clustering.clusters = { {
        .id = 17,
        .vertex_indices = { 0, 1, 2 },
        .local_triangles = { { 0, 1, 2 } },
        .uvs = {},
        .texture_id = std::nullopt,
        .absolute_error = 0.0,
    } };
    return batch;
}

} // namespace

TEST_CASE("DAG storage reopens a published batch and reads its metadata without the clustering file", "[dag_builder][storage]")
{
    const dag::ClusterBatch batch = sample_batch();
    const octree::Id root = octree::Id::root();
    const TempDirectory directory("dag-storage");
    const std::filesystem::path path = directory.path() / "output";
    {
        dag::storage::Draft draft = REQUIRE_VALUE(dag::storage::create(path, octree::storage::default_layout()));
        REQUIRE_VALUE(draft.save(root, batch));
        REQUIRE_VALUE(dag::storage::publish(std::move(draft)));
    }
    CHECK(REQUIRE_VALUE(io::filesystem::is_regular_file(path / octree::storage::index_file_name)));
    CHECK(REQUIRE_VALUE(io::filesystem::is_regular_file(path / octree::storage::metadata_file_name)));

    const dag::storage::Storage reopened = REQUIRE_VALUE(dag::storage::open(path));
    CHECK(reopened.has(root));
    REQUIRE_VALUE(reopened.load(root));
    const std::vector<std::filesystem::path> full_paths = reopened.paths(root);
    REQUIRE(full_paths.size() == 2);

    const dag::storage::MetadataStorage metadata_storage = REQUIRE_VALUE(dag::storage::open_metadata(path));
    REQUIRE(REQUIRE_VALUE(io::filesystem::remove(full_paths.front())));
    const dag::NodeMetadata metadata = REQUIRE_VALUE(metadata_storage.load(root));
    CHECK(metadata.group_assignment == batch.metadata.group_assignment);
    CHECK(metadata.groups.front().error == batch.metadata.groups.front().error);
}

TEST_CASE("DAG storage hard-links both files for the same format", "[dag_builder][storage]")
{
    const TempDirectory directory("dag-link");
    dag::storage::Draft source = REQUIRE_VALUE(dag::storage::create(directory.path() / "source", octree::storage::default_layout()));
    dag::storage::Draft target = REQUIRE_VALUE(dag::storage::create(directory.path() / "target", octree::storage::default_layout()));
    const octree::Id root = octree::Id::root();
    REQUIRE_VALUE(source.save(root, sample_batch()));
    REQUIRE_VALUE(target.link_from(root, source));

    const std::vector<std::filesystem::path> source_paths = source.paths(root);
    const std::vector<std::filesystem::path> target_paths = target.paths(root);
    REQUIRE(source_paths.size() == 2);
    REQUIRE(target_paths.size() == 2);
    CHECK(REQUIRE_VALUE(io::filesystem::equivalent(source_paths[0], target_paths[0])));
    CHECK(REQUIRE_VALUE(io::filesystem::equivalent(source_paths[1], target_paths[1])));
}
