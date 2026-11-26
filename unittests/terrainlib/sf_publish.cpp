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

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "hierarchy/NodeStatus.h"
#include "io/filesystem.h"
#include "mesh/SimpleMesh.h"
#include "mesh/storage.h"
#include "octree/Id.h"
#include "octree/path_layout.h"
#include "octree/storage/index_format.h"
#include "octree/storage/metadata.h"
#include "sf/publish.h"
#include "sf/validate_index.h"
#include "store/open.h"

namespace {

mesh::Simple sample_mesh() { return mesh::Simple({ { 0, 1, 2 } }, { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 } }); }

} // namespace

TEST_CASE("SF publication writes and validates a valid index", "[sf][publish]")
{
    const TempDirectory directory("sf-publish");
    const std::filesystem::path path = directory.path() / "output";
    mesh::storage::Draft draft = REQUIRE_VALUE(mesh::storage::create(path, octree::path_layout::flat::layout(), ".sfmesh"));
    REQUIRE_VALUE(draft.save(octree::Id::root(), sample_mesh()));

    REQUIRE_VALUE(sf::publish(std::move(draft)));
    CHECK(REQUIRE_VALUE(io::filesystem::is_regular_file(path / octree::storage::index_file_name)));
    CHECK(REQUIRE_VALUE(io::filesystem::is_regular_file(path / octree::storage::metadata_file_name)));
}

TEST_CASE("SF publication retains an invalid written index for diagnosis", "[sf][publish]")
{
    const TempDirectory directory("sf-publish-invalid");
    const std::filesystem::path path = directory.path() / "output";
    mesh::storage::Draft draft = REQUIRE_VALUE(mesh::storage::create(path, octree::path_layout::flat::layout(), ".sfmesh"));
    const std::filesystem::path draft_path = draft.base_path();
    const octree::Id root = octree::Id::root();
    REQUIRE_VALUE(draft.save(root, sample_mesh()));
    REQUIRE_VALUE(draft.save(root.child(0).value(), sample_mesh()));

    const auto error = REQUIRE_ERROR(sf::publish(std::move(draft)));

    CHECK(error.holds_code(sf::UnexpectedInnerNode { root }));
    CHECK(REQUIRE_VALUE(io::filesystem::is_regular_file(draft_path / octree::storage::index_file_name)));
    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(path)));

    const mesh::storage::Storage reopened = REQUIRE_VALUE(mesh::storage::open(draft_path, store::Accept::Draft));
    CHECK(reopened.index().status(root) == hierarchy::NodeStatus::Inner);
}
