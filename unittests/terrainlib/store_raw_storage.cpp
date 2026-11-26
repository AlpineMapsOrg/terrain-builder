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
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "common_error_codes.h"
#include "io/filesystem.h"
#include "octree/Id.h"
#include "octree/StoreTraits.h"
#include "octree/path_layout.h"
#include "store/RawStorage.h"
#include "store/path_layout.h"
#include "store_test_utils.h"

namespace {

using RawStorage = store::RawStorage<octree::KeyTraits, int>;

RawStorage make_raw_storage(const std::filesystem::path& path, std::unique_ptr<store::Codec<int>> codec)
{
    return RawStorage(store::path_layout::Resolver<octree::Id>(path, octree::path_layout::flat::layout()), std::move(codec));
}

} // namespace

TEST_CASE("has requires every codec file", "[store][RawStorage]")
{
    const TempDirectory directory("raw-multi-file");
    const RawStorage storage = make_raw_storage(directory.path(), std::make_unique<store_test::MultiFileCodec>());
    const octree::Id root = octree::Id::root();
    const std::vector<std::filesystem::path> paths = storage.paths(root);
    REQUIRE(paths.size() == 2);

    REQUIRE_VALUE(store_test::write_int(1, paths[0]));
    CHECK_FALSE(REQUIRE_VALUE(storage.has(root)));
    REQUIRE_VALUE(store_test::write_int(2, paths[1]));
    CHECK(REQUIRE_VALUE(storage.has(root)));
}

TEST_CASE("link_from hard-links every codec file", "[store][RawStorage]")
{
    const octree::Id root = octree::Id::root();

    SECTION("one file")
    {
        const TempDirectory source_directory("raw-link-one-source");
        const TempDirectory target_directory("raw-link-one-target");
        const RawStorage source = make_raw_storage(source_directory.path(), std::make_unique<store_test::IntCodec>());
        const RawStorage target = make_raw_storage(target_directory.path(), std::make_unique<store_test::IntCodec>());
        REQUIRE_VALUE(source.save(root, 41));

        REQUIRE_VALUE(target.link_from(root, source));
        CHECK(REQUIRE_VALUE(target.load(root)) == 41);
        CHECK(REQUIRE_VALUE(io::filesystem::equivalent(source.paths(root).front(), target.paths(root).front())));
    }

    SECTION("multiple files")
    {
        const TempDirectory source_directory("raw-link-many-source");
        const TempDirectory target_directory("raw-link-many-target");
        const RawStorage source = make_raw_storage(source_directory.path(), std::make_unique<store_test::MultiFileCodec>());
        const RawStorage target = make_raw_storage(target_directory.path(), std::make_unique<store_test::MultiFileCodec>());
        REQUIRE_VALUE(source.save(root, 42));

        REQUIRE_VALUE(target.link_from(root, source));
        const std::vector<std::filesystem::path> source_paths = source.paths(root);
        const std::vector<std::filesystem::path> target_paths = target.paths(root);
        REQUIRE(source_paths.size() == 2);
        REQUIRE(target_paths.size() == 2);
        CHECK(REQUIRE_VALUE(io::filesystem::equivalent(source_paths[0], target_paths[0])));
        CHECK(REQUIRE_VALUE(io::filesystem::equivalent(source_paths[1], target_paths[1])));
    }
}

TEST_CASE("link_from rejects a source with a different codec", "[store][RawStorage]")
{
    const TempDirectory source_directory("raw-link-codec-source");
    const TempDirectory target_directory("raw-link-codec-target");
    const RawStorage source = make_raw_storage(source_directory.path(), std::make_unique<store_test::MultiFileCodec>());
    const RawStorage target = make_raw_storage(target_directory.path(), std::make_unique<store_test::IntCodec>());
    const octree::Id root = octree::Id::root();
    REQUIRE_VALUE(source.save(root, 43));

    const auto error = REQUIRE_ERROR(target.link_from(root, source));
    CHECK(error.holds<Unsupported>());
    CHECK_FALSE(REQUIRE_VALUE(target.has(root)));
}
