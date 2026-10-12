/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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

#include <atomic>
#include <cstring>
#include <filesystem>
#include <future>
#include <latch>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include "../temporary_directory.h"
#include "io/byte.h"
#include "io/utils.h"
#include "octree/StoreTraits.h"
#include "octree/store_layout/Mappings.h"
#include "raster_store/StoreTraits.h"
#include "store/IndexedStorage.h"
#include "store/ThreadSafeStorage.h"
#include "string_utils.h"

namespace {

using test::TemporaryDirectory;

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
    ++part;
    const auto x = from_chars<uint32_t>(part->string());
    ++part;
    const auto y = from_chars<uint32_t>(part->string());
    if (!zoom.has_value() || !x.has_value() || !y.has_value()) {
        return std::nullopt;
    }
    const radix::tile::Id key { zoom.value(), { x.value(), y.value() } };
    return raster_store::StoreTraits::is_valid(key) ? std::optional<radix::tile::Id>(key) : std::nullopt;
}

template <typename Traits>
store::path_layout::Mapping<typename Traits::Key> test_mapping();

template <>
store::path_layout::Mapping<octree::Id> test_mapping<octree::StoreTraits>()
{
    return octree::store_layout::flat();
}

template <>
store::path_layout::Mapping<radix::tile::Id> test_mapping<raster_store::StoreTraits>()
{
    return { "test_tiles", tile_to_path, path_to_tile };
}

Expected<int> read_int(const std::filesystem::path& path)
{
    auto bytes = io::byte::read(path);
    if (!bytes) {
        return Error::propagate(std::move(bytes), "read test integer");
    }
    if (bytes->size() != sizeof(int)) {
        return Error::fail(Error::Code::CorruptData, "test integer has the wrong byte count");
    }
    int value;
    std::memcpy(&value, bytes->data(), sizeof(value));
    return value;
}

Expected<void> write_int(const int value, const std::filesystem::path& path)
{
    const auto bytes = std::as_bytes(std::span(&value, 1));
    auto written = io::byte::write(bytes, path, io::byte::WriteMode::CreateNew);
    if (!written) {
        return Error::propagate(std::move(written), "write test integer");
    }
    return {};
}

class IntCodec final : public store::Codec<int> {
public:
    std::vector<std::filesystem::path> paths(const std::filesystem::path& node_path) const override { return { add_extension(node_path, ".data") }; }

    Expected<int> read(const std::filesystem::path& node_path) const override { return read_int(paths(node_path).front()); }

    Expected<void> write(const std::filesystem::path& node_path, const int& value) const override { return write_int(value, paths(node_path).front()); }
};

class MultiFileCodec final : public store::Codec<int> {
public:
    std::vector<std::filesystem::path> paths(const std::filesystem::path& node_path) const override
    {
        return { add_extension(node_path, ".data"), add_extension(node_path, ".metadata") };
    }

    Expected<int> read(const std::filesystem::path& node_path) const override
    {
        auto value = read_int(paths(node_path).front());
        if (!value) {
            return Error::propagate(std::move(value), "read multi-file test value");
        }
        return value.value();
    }

    Expected<void> write(const std::filesystem::path& node_path, const int& value) const override
    {
        const auto node_paths = paths(node_path);
        if (!write_int(value, node_paths[0]) || !write_int(value + 1, node_paths[1])) {
            return Error::fail(Error::Code::Io, "multi-file test write failed");
        }
        return {};
    }
};

std::atomic_uint32_t index_writes = 0;

template <typename Traits>
Expected<store::IndexMetadata<Traits>> unused_index_read(const std::filesystem::path& path)
{
    return Error::fail(Error::Code::NotFound, "read unused test index from", path);
}

template <typename Traits>
Expected<void> count_index_write(const std::filesystem::path&, const store::IndexMetadata<Traits>&)
{
    ++index_writes;
    return {};
}

template <typename Traits>
std::optional<store::path_layout::Mapping<typename Traits::Key>> test_mapping_from_id(const std::string_view id)
{
    const auto mapping = test_mapping<Traits>();
    return id == mapping.id ? std::optional(mapping) : std::nullopt;
}

template <typename Traits>
store::IndexFormat<Traits> counting_index_format()
{
    return {
        "test.storeindex",
        unused_index_read<Traits>,
        count_index_write<Traits>,
        test_mapping_from_id<Traits>,
        test_mapping<Traits>,
    };
}

template <typename Traits = octree::StoreTraits>
store::IndexedStorage<Traits, int> make_indexed_storage(
    const std::filesystem::path& path, std::unique_ptr<store::Codec<int>> codec = std::make_unique<IntCodec>())
{
    const auto mapping = test_mapping<Traits>();
    return store::IndexedStorage<Traits, int>(
        store::RawStorage<Traits, int>(store::path_layout::Resolver<typename Traits::Key>(path, mapping), std::move(codec)),
        store::Index<Traits> {},
        typename store::Storage<Traits, int>::Persistence {
            counting_index_format<Traits>(),
            path / "test.storeindex",
            std::string(mapping.id),
            "test.Int",
            ".data",
        });
}

std::vector<octree::Id> root_children()
{
    const auto children = octree::Id::root().children().value();
    return { children.begin(), children.end() };
}

} // namespace

TEST_CASE("parent directory creation reports success for new and existing directories", "[io][bytes]")
{
    TemporaryDirectory directory("create-parents");
    const auto path = directory.path() / "parent" / "nested" / "payload";
    REQUIRE(io::utils::create_parent_directories(path));
    CHECK(std::filesystem::is_directory(path.parent_path()));
    CHECK_FALSE(std::filesystem::exists(path));
    REQUIRE(io::utils::create_parent_directories(path));
}

TEST_CASE("byte writes report directory creation errors", "[io][bytes]")
{
    TemporaryDirectory directory("write-parent-error");
    const std::filesystem::path blocker = directory.path() / "blocker";
    REQUIRE(write_int(1, blocker).has_value());

    const auto result = io::byte::write(io::byte::span {}, blocker / "payload", io::byte::WriteMode::Overwrite);
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code() == Error::Code::Io);
}

TEST_CASE("byte replace swaps in a new file and leaves hard links to the old one alone", "[io][bytes]")
{
    TemporaryDirectory directory("write-replace");
    const auto path = directory.path() / "index";
    const auto temporary_path = directory.path() / "index.tmp";
    const auto link = directory.path() / "link";
    const io::byte::vector first { std::byte { 1 }, std::byte { 2 }, std::byte { 3 } };
    const io::byte::vector second { std::byte { 4 }, std::byte { 5 } };
    const io::byte::vector stale { std::byte { 6 }, std::byte { 7 }, std::byte { 8 }, std::byte { 9 } };

    REQUIRE(io::byte::write(first, path, io::byte::WriteMode::Replace));
    CHECK(*io::byte::read(path) == first);
    std::filesystem::create_hard_link(path, link);
    // A temporary file left behind by a crash is truncated.
    REQUIRE(io::byte::write(stale, temporary_path, io::byte::WriteMode::CreateNew));

    REQUIRE(io::byte::write(second, path, io::byte::WriteMode::Replace));
    CHECK(*io::byte::read(path) == second);
    CHECK(*io::byte::read(link) == first);
    CHECK_FALSE(std::filesystem::exists(temporary_path));
}

TEMPLATE_TEST_CASE("shared storage has no hidden octree dependency", "[store][storage]", octree::StoreTraits, raster_store::StoreTraits)
{
    using Traits = TestType;
    TemporaryDirectory directory("traits");
    auto storage = make_indexed_storage<Traits>(directory.path());
    const auto root = Traits::root();

    CHECK_FALSE(storage.has(root).value());
    REQUIRE(storage.save(root, 42).has_value());
    CHECK(storage.has(root).value());
    REQUIRE(storage.load(root).has_value());
    CHECK(storage.load(root).value() == 42);
}

TEST_CASE("shared storage rejects invalid 2D keys", "[store][storage]")
{
    TemporaryDirectory directory("invalid-key");
    auto storage = make_indexed_storage<raster_store::StoreTraits>(directory.path());
    const radix::tile::Id invalid { 3, { 8, 0 } };

    CHECK(storage.load(invalid).error().code() == Error::Code::InvalidInput);
    CHECK(storage.save(invalid, 1).error().code() == Error::Code::InvalidInput);
    CHECK(storage.has(invalid).error().code() == Error::Code::InvalidInput);
}

TEST_CASE("shared storage rejects saving existing nodes", "[store][storage]")
{
    TemporaryDirectory directory("overwrite");
    auto storage = make_indexed_storage(directory.path());
    const octree::Id root = octree::Id::root();

    REQUIRE(storage.save(root, 1).has_value());
    const auto rejected = storage.save(root, 2);
    REQUIRE_FALSE(rejected.has_value());
    CHECK(rejected.error().code() == Error::Code::AlreadyExists);
    CHECK(storage.load(root).value() == 1);
}

TEST_CASE("raw storage requires every codec path", "[store][storage]")
{
    TemporaryDirectory directory("multi-file");
    store::RawStorage<octree::StoreTraits, int> storage(
        store::path_layout::Resolver<octree::Id>(directory.path(), octree::store_layout::flat()), std::make_unique<MultiFileCodec>());
    const octree::Id root = octree::Id::root();
    const auto paths = storage.paths(root).value();
    REQUIRE(paths.size() == 2);

    REQUIRE(write_int(1, paths[0]).has_value());
    CHECK_FALSE(storage.has(root).value());
    REQUIRE(write_int(2, paths[1]).has_value());
    CHECK(storage.has(root).value());
}

TEST_CASE("written payloads become available after exactly one commit", "[store][storage][write]")
{
    TemporaryDirectory directory("uncommitted");
    auto storage = make_indexed_storage(directory.path());
    const octree::Id root = octree::Id::root();

    REQUIRE(storage.write_payload(root, 7).has_value());
    CHECK_FALSE(storage.has(root).value());
    const auto uncommitted = storage.load(root);
    REQUIRE_FALSE(uncommitted.has_value());
    CHECK(uncommitted.error().code() == Error::Code::NotFound);

    REQUIRE(storage.commit(root).has_value());
    CHECK(storage.has(root).value());
    CHECK(storage.load(root).value() == 7);

    const auto recommitted = storage.commit(root);
    REQUIRE_FALSE(recommitted.has_value());
    CHECK(recommitted.error().code() == Error::Code::AlreadyExists);
}

TEST_CASE("payload writes reject linked nodes and keep the source file", "[store][storage][write][link]")
{
    TemporaryDirectory source_directory("write-linked-source");
    TemporaryDirectory target_directory("write-linked-target");
    auto source = make_indexed_storage(source_directory.path());
    auto target = make_indexed_storage(target_directory.path());
    const octree::Id root = octree::Id::root();
    REQUIRE(source.save(root, 41).has_value());
    REQUIRE(target.link_from(root, source).has_value());

    const auto rejected = target.write_payload(root, 2);
    REQUIRE_FALSE(rejected.has_value());
    CHECK(rejected.error().code() == Error::Code::AlreadyExists);
    CHECK(source.load(root).value() == 41);
    CHECK(target.load(root).value() == 41);
}

TEST_CASE("thread-safe storage saves different keys concurrently and one key once", "[store][storage][write]")
{
    TemporaryDirectory directory("thread-safe");
    store::ThreadSafeStorage storage(make_indexed_storage(directory.path()));

    SECTION("different keys")
    {
        const auto keys = root_children();
        std::vector<std::future<Expected<void>>> saves;
        for (std::size_t index = 0; index < keys.size(); ++index) {
            saves.push_back(std::async(std::launch::async, [&storage, &keys, index] { return storage.save(keys[index], int(index)); }));
        }
        for (auto& save : saves) {
            REQUIRE(save.get().has_value());
        }
        for (std::size_t index = 0; index < keys.size(); ++index) {
            CHECK(storage.load(keys[index]).value() == int(index));
        }
    }

    SECTION("same key")
    {
        const octree::Id root = octree::Id::root();
        constexpr int writers = 8;
        std::latch start(writers);
        std::vector<std::future<Expected<void>>> saves;
        for (int value = 0; value < writers; ++value) {
            saves.push_back(std::async(std::launch::async, [&storage, &start, root, value] {
                start.arrive_and_wait();
                return storage.save(root, value);
            }));
        }
        std::vector<int> saved;
        for (int value = 0; value < writers; ++value) {
            const auto result = saves[std::size_t(value)].get();
            if (result) {
                saved.push_back(value);
            } else {
                CHECK(result.error().code() == Error::Code::AlreadyExists);
            }
        }
        REQUIRE(saved.size() == 1);
        CHECK(storage.load(root).value() == saved.front());
    }
}

TEST_CASE("storage moves transfer and finalize dirty index state", "[store][storage]")
{
    index_writes = 0;
    TemporaryDirectory first_directory("move-first");
    TemporaryDirectory second_directory("move-second");
    TemporaryDirectory third_directory("move-third");

    {
        auto original = make_indexed_storage(first_directory.path());
        REQUIRE(original.save(octree::Id::root(), 1).has_value());
        auto moved = std::move(original);
        CHECK(index_writes == 0);
        REQUIRE(moved.save_index().has_value());
        CHECK(index_writes == 1);
    }
    CHECK(index_writes == 1);

    {
        auto destination = make_indexed_storage(second_directory.path());
        REQUIRE(destination.save(octree::Id::root(), 2).has_value());
        auto source = make_indexed_storage(third_directory.path());
        REQUIRE(source.save(octree::Id::root(), 3).has_value());
        destination = std::move(source);
        CHECK(index_writes == 2);
    }
    CHECK(index_writes == 3);
}

TEST_CASE("link_from hard-links every codec file", "[store][storage][link]")
{
    const octree::Id root = octree::Id::root();

    SECTION("one file")
    {
        TemporaryDirectory source_directory("link-one-source");
        TemporaryDirectory target_directory("link-one-target");
        auto source = make_indexed_storage(source_directory.path());
        auto target = make_indexed_storage(target_directory.path());
        REQUIRE(source.save(root, 41).has_value());

        REQUIRE(target.link_from(root, source).has_value());
        REQUIRE(target.has(root).value());
        CHECK(target.load(root).value() == 41);
        CHECK(std::filesystem::equivalent(source.paths(root)->front(), target.paths(root)->front()));
    }

    SECTION("multiple files")
    {
        TemporaryDirectory source_directory("link-many-source");
        TemporaryDirectory target_directory("link-many-target");
        auto source = make_indexed_storage(source_directory.path(), std::make_unique<MultiFileCodec>());
        auto target = make_indexed_storage(target_directory.path(), std::make_unique<MultiFileCodec>());
        REQUIRE(source.save(root, 42).has_value());

        REQUIRE(target.link_from(root, source).has_value());
        const auto source_paths = source.paths(root).value();
        const auto target_paths = target.paths(root).value();
        REQUIRE(source_paths.size() == 2);
        REQUIRE(target_paths.size() == 2);
        CHECK(std::filesystem::equivalent(source_paths[0], target_paths[0]));
        CHECK(std::filesystem::equivalent(source_paths[1], target_paths[1]));
    }
}

TEST_CASE("link_from rejects existing targets, missing sources and different codecs", "[store][storage][link]")
{
    const octree::Id root = octree::Id::root();

    SECTION("existing target")
    {
        TemporaryDirectory source_directory("link-existing-source");
        TemporaryDirectory target_directory("link-existing-target");
        auto source = make_indexed_storage(source_directory.path());
        auto target = make_indexed_storage(target_directory.path());
        REQUIRE(source.save(root, 44).has_value());
        REQUIRE(target.save(root, 1).has_value());

        const auto linked = target.link_from(root, source);
        REQUIRE_FALSE(linked.has_value());
        CHECK(linked.error().code() == Error::Code::AlreadyExists);
        CHECK(target.load(root).value() == 1);
    }

    SECTION("missing source")
    {
        TemporaryDirectory source_directory("link-missing-source");
        TemporaryDirectory target_directory("link-missing-target");
        auto source = make_indexed_storage(source_directory.path());
        auto target = make_indexed_storage(target_directory.path());

        const auto linked = target.link_from(root, source);
        REQUIRE_FALSE(linked.has_value());
        CHECK(linked.error().code() == Error::Code::NotFound);
        CHECK_FALSE(target.has(root).value());
    }

    SECTION("different codecs")
    {
        TemporaryDirectory source_directory("link-codec-source");
        TemporaryDirectory target_directory("link-codec-target");
        auto source = make_indexed_storage(source_directory.path(), std::make_unique<MultiFileCodec>());
        auto target = make_indexed_storage(target_directory.path());
        REQUIRE(source.save(root, 43).has_value());

        const auto linked = target.link_from(root, source);
        REQUIRE_FALSE(linked.has_value());
        CHECK(linked.error().code() == Error::Code::Unsupported);
        CHECK_FALSE(target.has(root).value());
    }
}
