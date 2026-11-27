#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include <fmt/format.h>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "error/format.h"
#include "io/FileError.h"
#include "io/bytes.h"
#include "octree/Id.h"
#include "octree/storage/MeshStorage.h"
#include "octree/storage/open.h"

TEST_CASE("Loading a node the index does not list fails as a missing file", "[octree][storage]") {
    const TempDirectory directory("octree_storage_index_miss");
    const octree::IndexedMeshStorage storage = REQUIRE_VALUE(octree::open_folder_indexed(directory.path()));
    const octree::Id id = octree::Id::root();

    const auto result = storage.load(id);

    REQUIRE(!result.has_value());
    CHECK(result.error().holds_code(io::make_file_not_found(storage.path_for(id))));
}

TEST_CASE("Copying a node the source index does not list fails as a missing file", "[octree][storage]") {
    const TempDirectory directory("octree_storage_copy_index_miss");
    const octree::IndexedMeshStorage source = REQUIRE_VALUE(octree::open_folder_indexed(directory.path() / "source"));
    octree::MeshStorage target = REQUIRE_VALUE(octree::open_folder(directory.path() / "target"));
    const octree::Id id = octree::Id::root();

    const auto result = target.copy_from(id, source);

    REQUIRE(!result.has_value());
    CHECK(result.error().holds_code(io::make_file_not_found(source.path_for(id))));
}

TEST_CASE("A failed node copy names the file it could not replace", "[octree][storage]") {
    const TempDirectory directory("octree_storage_failed_copy_step");
    const octree::MeshStorage source = REQUIRE_VALUE(octree::open_folder(directory.path() / "source"));
    octree::MeshStorage target = REQUIRE_VALUE(octree::open_folder(directory.path() / "target"));
    target.settings().allow_overwrite = true;
    const octree::Id id = octree::Id::root();

    REQUIRE(io::write_bytes_to_path(std::vector<uint8_t>{1}, source.path_for(id)).has_value());
    std::filesystem::create_directories(target.path_for(id) / "child");

    const auto result = target.copy_from(id, source);

    REQUIRE(!result.has_value());
    const io::FileError *error = result.error().get_if<io::FileError>();
    REQUIRE(error != nullptr);
    CHECK(error->path() == target.path_for(id));
    // A link onto the directory fails as file_exists, so another code shows the removal reported its own failure.
    CHECK(error->code() != std::errc::file_exists);
}
