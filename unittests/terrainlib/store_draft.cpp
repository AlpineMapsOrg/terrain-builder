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
#include <future>
#include <latch>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "common_error_codes.h"
#include "enumerate.h"
#include "hierarchy/NodeStatus.h"
#include "io/filesystem.h"
#include "octree/Id.h"
#include "octree/StoreTraits.h"
#include "octree/path_layout.h"
#include "raster_store/StoreTraits.h"
#include "store/Draft.h"
#include "store/IndexFormat.h"
#include "store/SimpleIndex.h"
#include "store/create.h"
#include "store/index_checks.h"
#include "store/inputs.h"
#include "store/open.h"
#include "store/path_layout.h"
#include "store_test_utils.h"
#include "string_utils.h"

namespace {

std::filesystem::path tile_to_path(const radix::tile::Id& key)
{
    return std::to_string(key.zoom_level) + "/" + std::to_string(key.coords.x) + "/" + std::to_string(key.coords.y);
}

std::optional<radix::tile::Id> path_to_tile(const std::filesystem::path& node_path)
{
    if (node_path.is_absolute() || std::distance(node_path.begin(), node_path.end()) != 3) {
        return std::nullopt;
    }
    auto part = node_path.begin();
    const auto zoom = from_chars<unsigned>(part->string());
    part++;
    const auto x = from_chars<uint32_t>(part->string());
    part++;
    const auto y = from_chars<uint32_t>(part->string());
    if (!zoom.has_value() || !x.has_value() || !y.has_value()) {
        return std::nullopt;
    }
    const radix::tile::Id key { zoom.value(), { x.value(), y.value() } };
    return raster_store::KeyTraits::is_valid(key) ? std::optional<radix::tile::Id>(key) : std::nullopt;
}

template <typename Traits>
store::path_layout::Layout<typename Traits::Key> test_layout();

template <>
store::path_layout::Layout<octree::Id> test_layout<octree::KeyTraits>()
{
    return octree::path_layout::flat::layout();
}

template <>
store::path_layout::Layout<radix::tile::Id> test_layout<raster_store::KeyTraits>()
{
    return { "test_tiles", tile_to_path, path_to_tile };
}

template <typename Traits>
typename Traits::Key test_key();

template <>
octree::Id test_key<octree::KeyTraits>()
{
    return octree::Id::root();
}

template <>
radix::tile::Id test_key<raster_store::KeyTraits>()
{
    return { 0, { 0, 0 } };
}

template <typename Traits>
alp::Result<store::SimpleIndex<Traits>, store::ReadIndexError> read_no_index(const std::filesystem::path&)
{
    return alp::make_error(Unsupported {}, "the test format cannot read an index");
}

template <typename Traits>
alp::Result<void, store::WriteIndexError> write_no_index(const std::filesystem::path&, const store::SimpleIndex<Traits>&)
{
    return {};
}

template <typename Traits>
std::optional<store::path_layout::Layout<typename Traits::Key>> resolve_test_layout(const std::string_view id)
{
    const auto layout = test_layout<Traits>();
    return id == layout.id ? std::optional(layout) : std::nullopt;
}

// A format that keeps the index in memory only.
template <typename Traits>
store::IndexFormat<store::SimpleIndex<Traits>> unsaved_index_format()
{
    return {
        read_no_index<Traits>,
        write_no_index<Traits>,
        resolve_test_layout<Traits>,
        test_layout<Traits>,
    };
}

std::vector<octree::Id> root_children()
{
    const auto children = octree::Id::root().children().value();
    return { children.begin(), children.end() };
}

} // namespace

TEMPLATE_TEST_CASE("Draft has no hidden octree dependency", "[store][Draft]", octree::KeyTraits, raster_store::KeyTraits)
{
    using Traits = TestType;
    using Index = store::SimpleIndex<Traits>;
    const TempDirectory directory("draft-traits");
    auto create_result = store::create<Index, int>(directory.path() / "draft", unsaved_index_format<Traits>(), test_layout<Traits>(), std::make_unique<store_test::IntCodec>());
    store::Draft<Index, int> draft = REQUIRE_VALUE(std::move(create_result));
    const auto key = test_key<Traits>();

    CHECK_FALSE(draft.has(key));
    REQUIRE_VALUE(draft.save(key, 42));
    CHECK(draft.has(key));
    CHECK(REQUIRE_VALUE(draft.load(key)) == 42);
}

TEST_CASE("save rejects existing nodes", "[store][Draft]")
{
    const TempDirectory directory("draft-overwrite");
    store_test::Draft draft = REQUIRE_VALUE(store_test::create_draft(directory.path() / "draft"));
    const octree::Id root = octree::Id::root();

    REQUIRE_VALUE(draft.save(root, 1));
    const auto error = REQUIRE_ERROR(draft.save(root, 2));
    CHECK(error.holds<store::AlreadyExists>());
    CHECK(REQUIRE_VALUE(draft.load(root)) == 1);
}

TEST_CASE("save rejects linked nodes and keeps the source file", "[store][Draft]")
{
    const TempDirectory directory("draft-write-linked");
    store_test::Draft source = REQUIRE_VALUE(store_test::create_draft(directory.path() / "source"));
    store_test::Draft target = REQUIRE_VALUE(store_test::create_draft(directory.path() / "target"));
    const octree::Id root = octree::Id::root();
    REQUIRE_VALUE(source.save(root, 41));
    REQUIRE_VALUE(target.link_from(root, source));

    const auto error = REQUIRE_ERROR(target.save(root, 2));
    CHECK(error.holds<store::AlreadyExists>());
    CHECK(REQUIRE_VALUE(source.load(root)) == 41);
    CHECK(REQUIRE_VALUE(target.load(root)) == 41);
}

TEST_CASE("save stores different keys concurrently and one key once", "[store][Draft]")
{
    const TempDirectory directory("draft-concurrent");
    store_test::Draft draft = REQUIRE_VALUE(store_test::create_draft(directory.path() / "draft"));

    SECTION("different keys")
    {
        const std::vector<octree::Id> keys = root_children();
        std::vector<std::future<alp::Result<void, store_test::Draft::SaveError>>> saves;
        for (const auto [index, key] : enumerate<int>(keys)) {
            saves.push_back(std::async(std::launch::async, [&draft, key, index] { return draft.save(key, index); }));
        }
        for (auto& save : saves) {
            REQUIRE_VALUE(save.get());
        }
        for (const auto [index, key] : enumerate<int>(keys)) {
            CHECK(REQUIRE_VALUE(draft.load(key)) == index);
        }
    }

    SECTION("same key")
    {
        const octree::Id root = octree::Id::root();
        constexpr int writers = 8;
        std::latch start(writers);
        std::vector<std::future<alp::Result<void, store_test::Draft::SaveError>>> saves;
        for (int value = 0; value < writers; value++) {
            saves.push_back(std::async(std::launch::async, [&draft, &start, root, value] {
                start.arrive_and_wait();
                return draft.save(root, value);
            }));
        }
        std::vector<int> saved;
        for (const auto [value, save] : enumerate<int>(saves)) {
            const auto result = save.get();
            if (result.has_value()) {
                saved.push_back(value);
            } else {
                CHECK(result.error().holds<store::AlreadyExists>());
            }
        }
        REQUIRE(saved.size() == 1);
        CHECK(REQUIRE_VALUE(draft.load(root)) == saved.front());
    }
}

TEST_CASE("hash reports missing and virtual nodes", "[store][Draft]")
{
    const TempDirectory directory("draft-hash-lookup");
    store_test::Draft draft = REQUIRE_VALUE(store_test::create_draft(directory.path() / "draft"));
    const octree::Id root = octree::Id::root();
    REQUIRE_VALUE(draft.save(root.child(3).value(), 1));
    REQUIRE(draft.index().status(root) == hierarchy::NodeStatus::Virtual);

    CHECK(REQUIRE_ERROR(draft.hash(root)).holds<store::NotFound>());
    CHECK(REQUIRE_ERROR(draft.hash(root.child(1).value())).holds<store::NotFound>());
    CHECK(REQUIRE_VALUE(draft.hash(root.child(3).value())) == store_test::int_hash(1));
}

TEST_CASE("load rejects valid payloads that differ from the index", "[store][Draft]")
{
    const TempDirectory directory("draft-hash-mismatch");
    store_test::Draft draft = REQUIRE_VALUE(store_test::create_draft(directory.path() / "draft"));
    const octree::Id root = octree::Id::root();
    REQUIRE_VALUE(draft.save(root, 13));
    const std::filesystem::path node_file = draft.paths(root).front();
    REQUIRE(REQUIRE_VALUE(io::filesystem::remove(node_file)));
    REQUIRE_VALUE(store_test::write_int(14, node_file));

    CHECK(REQUIRE_ERROR(draft.load(root)).holds<store::CorruptData>());
}

TEST_CASE("moves and closing keep hashes", "[store][Draft]")
{
    const octree::Id root = octree::Id::root();
    const octree::Id child = root.child(4).value();
    const TempDirectory directory("draft-hash-move");
    const std::filesystem::path first_path = directory.path() / "first";

    {
        store_test::Draft original = REQUIRE_VALUE(store_test::create_draft(first_path));
        REQUIRE_VALUE(original.save(root, 30));
        store_test::Draft moved = std::move(original);
        CHECK(REQUIRE_VALUE(moved.hash(root)) == store_test::int_hash(30));

        store_test::Draft destination = REQUIRE_VALUE(store_test::create_draft(directory.path() / "second"));
        destination = std::move(moved);
        CHECK(REQUIRE_VALUE(destination.hash(root)) == store_test::int_hash(30));

        REQUIRE_VALUE(destination.save(child, 31));
        CHECK(REQUIRE_VALUE(destination.hash(child)) == store_test::int_hash(31));
    }

    // Closing the draft writes its index.
    const store_test::Storage reopened = REQUIRE_VALUE(store_test::open_storage(store::make_draft_path(first_path), store::Accept::Draft));
    CHECK(REQUIRE_VALUE(reopened.hash(root)) == store_test::int_hash(30));
    CHECK(REQUIRE_VALUE(reopened.hash(child)) == store_test::int_hash(31));
}

TEST_CASE("links transfer recorded hashes", "[store][Draft]")
{
    const TempDirectory directory("draft-hash-link");
    store_test::Draft source = REQUIRE_VALUE(store_test::create_draft(directory.path() / "source"));
    store_test::Draft target = REQUIRE_VALUE(store_test::create_draft(directory.path() / "target"));
    const octree::Id root = octree::Id::root();
    REQUIRE_VALUE(source.save(root, 5));

    REQUIRE_VALUE(target.link_from(root, source));
    CHECK(REQUIRE_VALUE(io::filesystem::equivalent(source.paths(root).front(), target.paths(root).front())));
    CHECK(REQUIRE_VALUE(target.hash(root)) == REQUIRE_VALUE(source.hash(root)));
}

TEST_CASE("link_from links from a storage, a draft and a checkpoint", "[store][Draft]")
{
    const TempDirectory directory("draft-link-sources");
    const store_test::Inputs inputs { "input", 1 };
    store_test::Draft source = REQUIRE_VALUE(store_test::create_draft(directory.path() / "source"));
    const octree::Id root = octree::Id::root();
    REQUIRE_VALUE(source.save(root, 41));
    REQUIRE_VALUE(source.save_index());
    REQUIRE_VALUE(store::inputs::write<store_test::InputsSchema>(source.base_path(), inputs));
    const std::filesystem::path target_path = directory.path() / "target";
    store_test::Draft target = REQUIRE_VALUE(store_test::create_draft(target_path));

    SECTION("storage")
    {
        const store_test::Storage storage = REQUIRE_VALUE(store_test::open_storage(source.base_path(), store::Accept::Draft));
        REQUIRE_VALUE(target.link_from(root, storage));
    }

    SECTION("draft")
    {
        REQUIRE_VALUE(target.link_from(root, source));
    }

    SECTION("checkpoint")
    {
        auto open_result = store::open_for_reuse<store_test::InputsSchema>(source.base_path(), target_path, inputs, store_test::open_storage);
        const store_test::Checkpoint checkpoint = REQUIRE_VALUE(std::move(open_result));
        REQUIRE_VALUE(target.link_from(root, checkpoint));
    }

    CHECK(REQUIRE_VALUE(target.load(root)) == 41);
    CHECK(REQUIRE_VALUE(io::filesystem::equivalent(source.paths(root).front(), target.paths(root).front())));
    CHECK(REQUIRE_VALUE(target.hash(root)) == REQUIRE_VALUE(source.hash(root)));
}

TEST_CASE("link_from rejects existing targets and missing sources", "[store][Draft]")
{
    const TempDirectory directory("draft-link-rejects");
    store_test::Draft source = REQUIRE_VALUE(store_test::create_draft(directory.path() / "source"));
    store_test::Draft target = REQUIRE_VALUE(store_test::create_draft(directory.path() / "target"));
    const octree::Id root = octree::Id::root();

    SECTION("existing target")
    {
        REQUIRE_VALUE(source.save(root, 44));
        REQUIRE_VALUE(target.save(root, 1));

        CHECK(REQUIRE_ERROR(target.link_from(root, source)).holds<store::AlreadyExists>());
        CHECK(REQUIRE_VALUE(target.load(root)) == 1);
    }

    SECTION("missing source")
    {
        CHECK(REQUIRE_ERROR(target.link_from(root, source)).holds<store::NotFound>());
        CHECK_FALSE(target.has(root));
    }

    SECTION("source files the source's index does not list")
    {
        REQUIRE_VALUE(store_test::write_int(4, source.paths(root).front()));

        CHECK(REQUIRE_ERROR(target.link_from(root, source)).holds<store::NotFound>());
        CHECK_FALSE(target.has(root));
    }

    SECTION("empty checkpoint")
    {
        const store_test::Checkpoint checkpoint {};

        CHECK(REQUIRE_ERROR(target.link_from(root, checkpoint)).holds<store::NotFound>());
        CHECK_FALSE(target.has(root));
    }

    SECTION("the draft itself")
    {
        REQUIRE_VALUE(target.save(root, 1));

        CHECK(REQUIRE_ERROR(target.link_from(root, target)).holds<store::AlreadyExists>());
        CHECK(REQUIRE_VALUE(target.load(root)) == 1);
    }
}
