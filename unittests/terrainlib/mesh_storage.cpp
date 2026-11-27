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
#include "io/filesystem.h"
#include "mesh/SimpleMesh.h"
#include "mesh/storage.h"
#include "octree/Id.h"
#include "octree/path_layout.h"
#include "octree/storage/index_format.h"
#include "octree/storage/metadata.h"
#include "store/Metadata.h"

namespace {

mesh::Simple triangle_mesh(const double offset)
{
    return mesh::Simple({ { 0, 1, 2 } }, { { offset, 0.0, 0.0 }, { offset + 1.0, 0.0, 0.0 }, { offset, 1.0, 0.0 } });
}

} // namespace

TEST_CASE("storage hard-links matching payload formats", "[mesh][storage]")
{
    const TempDirectory directory("hard-link");
    mesh::storage::Draft source = REQUIRE_VALUE(mesh::storage::create(directory.path() / "source", octree::path_layout::flat::layout(), ".sfmesh"));
    mesh::storage::Draft target = REQUIRE_VALUE(mesh::storage::create(directory.path() / "target", octree::path_layout::flat::layout(), ".sfmesh"));

    const octree::Id root = octree::Id::root();
    REQUIRE_VALUE(source.save(root, triangle_mesh(0.0)));
    REQUIRE_VALUE(target.link_from(root, source));
    CHECK(REQUIRE_VALUE(io::filesystem::equivalent(source.paths(root).front(), target.paths(root).front())));
    REQUIRE_VALUE(source.save_index());
    REQUIRE_VALUE(target.save_index());
}

TEST_CASE("create records the layout, payload class and codec that open uses", "[mesh][storage]")
{
    const TempDirectory directory("mesh-storage-open");
    const std::filesystem::path path = directory.path() / "storage";
    {
        mesh::storage::Draft draft = REQUIRE_VALUE(mesh::storage::create(path, octree::path_layout::flat::layout(), ".sfmesh"));
        REQUIRE_VALUE(draft.save(octree::Id::root(), triangle_mesh(2.0)));
        REQUIRE_VALUE(mesh::storage::publish(std::move(draft)));
    }
    CHECK(REQUIRE_VALUE(io::filesystem::is_regular_file(path / octree::storage::index_file_name)));
    CHECK(REQUIRE_VALUE(io::filesystem::is_regular_file(path / octree::storage::metadata_file_name)));

    const mesh::storage::Storage storage = REQUIRE_VALUE(mesh::storage::open(path));
    CHECK(storage.has(octree::Id::root()));

    const store::Metadata metadata = REQUIRE_VALUE(octree::storage::read_metadata(path));
    CHECK(metadata.layout_id == "flat");
    CHECK(metadata.payload_class == mesh::storage::payload_class);
    CHECK(metadata.codec_id == ".sfmesh");
}
