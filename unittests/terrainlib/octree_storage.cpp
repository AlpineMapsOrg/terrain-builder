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
#include <memory>
#include <string>
#include <system_error>

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "common_error_codes.h"
#include "io/FileError.h"
#include "io/bytes.h"
#include "io/filesystem.h"
#include "mesh/SimpleMesh.h"
#include "mesh/codec/resolve.h"
#include "mesh/storage.h"
#include "octree/Id.h"
#include "octree/path_layout.h"
#include "octree/storage/create.h"
#include "octree/storage/index_format.h"
#include "octree/storage/metadata.h"
#include "octree/storage/open.h"
#include "octree/storage/publish.h"
#include "store/Codec.h"
#include "store/Metadata.h"
#include "store/Storage.h"
#include "store/storage_paths.h"

namespace {

mesh::Simple triangle() { return mesh::Simple({ { 0, 1, 2 } }, { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 } }); }

alp::Result<store::Draft<octree::storage::Index, mesh::Simple>, octree::storage::CreateError> create_mesh_storage(const std::filesystem::path& final_path)
{
    std::unique_ptr<store::Codec<mesh::Simple>> codec = REQUIRE_VALUE(mesh::codec::resolve(".sfmesh"));
    return octree::storage::create<mesh::Simple>(final_path, std::string(mesh::storage::payload_class), octree::path_layout::flat::layout(), std::move(codec));
}

alp::Result<store::Storage<octree::storage::Index, mesh::Simple>, octree::storage::OpenError> open_mesh_storage(const std::filesystem::path& path)
{
    return octree::storage::open<mesh::Simple>(path, mesh::storage::payload_class, mesh::codec::resolve);
}

} // namespace

TEST_CASE("octree storages are created empty, published, and reopened", "[octree][storage][create][publish][open]")
{
    const TempDirectory directory("octree-storage");
    const std::filesystem::path final_path = directory.path() / "output";
    const octree::Id root = octree::Id::root();
    {
        store::Draft<octree::storage::Index, mesh::Simple> draft = REQUIRE_VALUE(create_mesh_storage(final_path));
        CHECK(draft.base_path() == store::make_draft_path(final_path));
        CHECK(REQUIRE_VALUE(io::filesystem::is_regular_file(draft.base_path() / octree::storage::index_file_name)));
        REQUIRE_VALUE(draft.save(root, triangle()));
        REQUIRE_VALUE(octree::storage::publish(std::move(draft)));
    }
    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(store::make_draft_path(final_path))));

    const store::Storage<octree::storage::Index, mesh::Simple> reopened = REQUIRE_VALUE(open_mesh_storage(final_path));
    CHECK(reopened.has(root));
    CHECK(reopened.codec_id() == ".sfmesh");

    const auto error = REQUIRE_ERROR(create_mesh_storage(final_path));
    CHECK(error.holds_code(io::FileError(std::errc::file_exists, final_path)));
}

TEST_CASE("octree opening rejects a mismatched metadata payload class", "[octree][storage][open]")
{
    const TempDirectory directory("payload-class");
    const store::Metadata metadata {
        .layout_id = "flat",
        .payload_class = "dag.ClusterBatch",
        .codec_id = ".sfmesh",
    };
    REQUIRE_VALUE(octree::storage::write_metadata(directory.path(), metadata));

    CHECK(REQUIRE_ERROR(open_mesh_storage(directory.path())).holds<store::UnexpectedPayloadClass>());
}

TEST_CASE("octree opening retains index failures without directory fallback", "[octree][storage][open]")
{
    const TempDirectory directory("open-errors");
    store::Metadata metadata {
        .layout_id = "unknown-layout",
        .payload_class = std::string(mesh::storage::payload_class),
        .codec_id = ".sfmesh",
    };
    REQUIRE_VALUE(octree::storage::write_metadata(directory.path(), metadata));

    const auto unknown_layout = REQUIRE_ERROR(open_mesh_storage(directory.path()));
    CHECK(unknown_layout.holds<Unsupported>());

    metadata.layout_id = "flat";
    metadata.codec_id = ".unknown";
    REQUIRE_VALUE(octree::storage::write_metadata(directory.path(), metadata));
    CHECK(REQUIRE_ERROR(open_mesh_storage(directory.path())).holds<Unsupported>());

    metadata.codec_id = ".sfmesh";
    REQUIRE_VALUE(octree::storage::write_metadata(directory.path(), metadata));
    const io::bytes::vector malformed_bytes { 0xff, 0x00, 0x01 };
    REQUIRE_VALUE(io::bytes::write(malformed_bytes, directory.path() / octree::storage::index_file_name, { .mode = io::WriteMode::Overwrite }));
    CHECK(REQUIRE_ERROR(open_mesh_storage(directory.path())).holds<store::CorruptIndex>());

    const store::Metadata independent_metadata = REQUIRE_VALUE(octree::storage::read_metadata(directory.path()));
    CHECK(independent_metadata == metadata);
}
