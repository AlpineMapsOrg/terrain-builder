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
#include <ranges>
#include <string>
#include <system_error>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_precision.hpp>
#include <radix/tile.h>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "common_error_codes.h"
#include "hierarchy/NodeStatus.h"
#include "io/FileError.h"
#include "io/WriteMode.h"
#include "io/bytes.h"
#include "io/envelope.h"
#include "io/filesystem.h"
#include "raster_store/Tile.h"
#include "raster_store/io/TileCodec.h"
#include "raster_store/attribution/table_file.h"
#include "raster_store/pixel.h"
#include "raster_store/storage.h"
#include "raster_store_test_utils.h"
#include "store/Metadata.h"
#include "store/index_checks.h"
#include "store/open.h"
#include "store/storage_paths.h"

namespace {

namespace attribution = raster_store::attribution;
namespace storage = raster_store::storage;
namespace tile_codec = raster_store::io::tile_codec;
using raster_store::io::TileCodec;
using raster_store_test::MetadataSchema;
using raster_store_test::check_bytes;
using raster_store_test::small_options;
using raster_store_test::write_table;


bool is_not_found(const io::FileError& error)
{
    return error.code() == std::errc::no_such_file_or_directory;
}

} // namespace

TEST_CASE("image tile storages are available only for RGB8 payloads", "[raster_store][storage]")
{
    const std::string codec_id = GENERATE("amoit", "amojt");
    CHECK(REQUIRE_ERROR(tile_codec::resolve<float>(codec_id, 4)).holds<Unsupported>());
    CHECK(REQUIRE_ERROR(tile_codec::resolve<glm::u8vec4>(codec_id, 4)).holds<Unsupported>());

    const TempDirectory directory("image-storage");
    write_table(directory.path());
    storage::CreateOptions options = small_options();
    options.codec_id = codec_id;
    CHECK(REQUIRE_ERROR(storage::create<float>(directory.path() / "scalar", options)).holds<Unsupported>());
    const std::filesystem::path final_path = directory.path() / "colour";
    storage::Draft<glm::u8vec3> draft = REQUIRE_VALUE(storage::create<glm::u8vec3>(final_path, options));
    raster_store::Tile<glm::u8vec3> tile(4);
    tile.data.fill(glm::u8vec3(255));
    tile.attribution.fill(1);
    REQUIRE_VALUE(draft.save({ 0, { 0, 0 } }, tile));
    REQUIRE_VALUE(storage::publish(std::move(draft)));
    CHECK(REQUIRE_VALUE(io::filesystem::exists(final_path / ("0/0/0." + codec_id))));

    const storage::Storage<glm::u8vec3> opened = REQUIRE_VALUE(storage::open<glm::u8vec3>(final_path));
    CHECK(REQUIRE_VALUE(storage::read_metadata(final_path)).ids.codec_id == codec_id);
    check_bytes(REQUIRE_VALUE(opened.load({ 0, { 0, 0 } })).attribution, tile.attribution);
}

TEST_CASE("AMOFT storages require Float32 and reopen with exact data", "[raster_store][storage]")
{
    CHECK(REQUIRE_ERROR(tile_codec::resolve<double>("amoft", 4)).holds<Unsupported>());
    CHECK(REQUIRE_ERROR(tile_codec::resolve<uint32_t>("amoft", 4)).holds<Unsupported>());
    CHECK(REQUIRE_ERROR(tile_codec::resolve<glm::vec3>("amoft", 4)).holds<Unsupported>());
    CHECK(REQUIRE_ERROR(tile_codec::resolve<glm::u8vec3>("amoft", 4)).holds<Unsupported>());

    const TempDirectory directory("amoft-storage");
    write_table(directory.path());
    storage::CreateOptions options = small_options();
    options.codec_id = "amoft";
    const std::filesystem::path final_path = directory.path() / "storage";
    storage::Draft<float> draft = REQUIRE_VALUE(storage::create<float>(final_path, options));
    raster_store::Tile<float> tile(4);
    tile.data.fill(-1.25f);
    REQUIRE_VALUE(draft.save({ 0, { 0, 0 } }, tile));
    REQUIRE_VALUE(storage::publish(std::move(draft)));

    CHECK(REQUIRE_VALUE(storage::read_metadata(final_path)).ids.codec_id == "amoft");
    const storage::Storage<float> opened = REQUIRE_VALUE(storage::open<float>(final_path));
    check_bytes(REQUIRE_VALUE(opened.load({ 0, { 0, 0 } })).data, tile.data);
}

TEST_CASE("vertex storages include the overlap in their stored tiles", "[raster_store][storage]")
{
    const TempDirectory directory("raster-vertex");
    write_table(directory.path());
    storage::CreateOptions options = small_options();
    options.halo_width = 1;
    options.placement = raster_store::pixel::Placement::Vertex;
    options.overlap = 1;
    const std::filesystem::path final_path = directory.path() / "storage";
    storage::Draft<float> draft = REQUIRE_VALUE(storage::create<float>(final_path, options));
    raster_store::Tile<float> tile(7);
    tile.data.fill(3.5f);
    REQUIRE_VALUE(draft.save({ 0, { 0, 0 } }, tile));
    REQUIRE_VALUE(storage::publish(std::move(draft)));

    const storage::Metadata metadata = REQUIRE_VALUE(storage::read_metadata(final_path));
    CHECK(metadata.nominal_tile_size == 4);
    CHECK(metadata.halo_width == 1);
    CHECK(metadata.overlap == 1);
    CHECK(metadata.stored_tile_size == 7);
    CHECK(metadata.placement == raster_store::pixel::Placement::Vertex);
    const storage::Storage<float> opened = REQUIRE_VALUE(storage::open<float>(final_path));
    check_bytes(REQUIRE_VALUE(opened.load({ 0, { 0, 0 } })).data, tile.data);
}

TEST_CASE("raster storages checkpoint metadata dimensions and publish explicitly", "[raster_store][storage]")
{
    const TempDirectory directory("raster-lifecycle");
    write_table(directory.path());
    const std::filesystem::path final_path = directory.path() / "storage";
    const std::filesystem::path draft_path = directory.path() / "storage.part";
    const radix::tile::Id key { 0, { 0, 0 } };
    storage::Draft<float> draft = REQUIRE_VALUE(storage::create<float>(final_path, small_options()));
    REQUIRE_ERROR(storage::open<float>(draft_path));
    REQUIRE_ERROR(storage::open<float>(draft_path / ""));
    REQUIRE_VALUE(storage::open<float>(draft_path, store::Accept::Draft));
    const io::bytes::vector metadata_before = REQUIRE_VALUE(io::bytes::read(draft_path / storage::metadata_file_name));
    raster_store::Tile<float> tile(4);
    tile.data.fill(12.5f);
    // Reference checks belong to RF builder, not storage reads or writes.
    tile.attribution.fill(65534);
    REQUIRE_VALUE(draft.save(key, tile));
    REQUIRE_VALUE(draft.save_index());
    CHECK(REQUIRE_VALUE(io::bytes::read(draft_path / storage::metadata_file_name)) == metadata_before);

    const storage::Storage<float> checkpoint = REQUIRE_VALUE(storage::open<float>(draft_path, store::Accept::Draft));
    REQUIRE_VALUE(checkpoint.load(key));
    CHECK(checkpoint.codec_id() == "amort");
    const storage::Metadata metadata = REQUIRE_VALUE(storage::read_metadata(draft_path));
    CHECK(metadata.stored_tile_size == 4);
    CHECK(metadata.nominal_tile_size == 4);
    CHECK(metadata.ids.payload_class == "float32");
    CHECK(metadata.ids.codec_id == "amort");

    REQUIRE_VALUE(storage::publish(std::move(draft)));
    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(draft_path)));
    const storage::Storage<float> published = REQUIRE_VALUE(storage::open<float>(final_path));
    CHECK(published.base_path() == final_path);
    check_bytes(REQUIRE_VALUE(published.load(key)).data, tile.data);
    CHECK(REQUIRE_ERROR(storage::open<uint32_t>(final_path)).holds<store::UnexpectedPayloadClass>());
}

TEST_CASE("failed checkpoints retain the previous index and ignore unindexed files", "[raster_store][storage]")
{
    const TempDirectory directory("raster-failed-checkpoint");
    write_table(directory.path());
    const radix::tile::Id key { 0, { 0, 0 } };
    storage::Draft<float> draft = REQUIRE_VALUE(storage::create<float>(directory.path() / "storage", small_options()));
    const std::filesystem::path draft_path = draft.base_path();
    REQUIRE_VALUE(draft.save(key, raster_store::Tile<float>(4)));
    REQUIRE(REQUIRE_VALUE(io::filesystem::create_directory(draft_path / "raster_store.index.tmp")));
    REQUIRE_ERROR(draft.save_index());

    const storage::Storage<float> checkpoint = REQUIRE_VALUE(storage::open<float>(draft_path, store::Accept::Draft));
    CHECK(std::ranges::empty(checkpoint.index().keys()));
    CHECK(REQUIRE_ERROR(checkpoint.load(key)).holds<store::NotFound>());

    REQUIRE(REQUIRE_VALUE(io::filesystem::remove(draft_path / "raster_store.index.tmp")));
    REQUIRE_VALUE(draft.save_index());
    const storage::Storage<float> reopened = REQUIRE_VALUE(storage::open<float>(draft_path, store::Accept::Draft));
    CHECK(std::ranges::distance(reopened.index().keys()) == 1);
}

TEST_CASE("raster publication does not scan payloads or reopen output", "[raster_store][storage]")
{
    const TempDirectory directory("raster-publish-no-scan");
    write_table(directory.path());
    const std::filesystem::path final_path = directory.path() / "storage";
    const radix::tile::Id key { 0, { 0, 0 } };
    storage::Draft<float> draft = REQUIRE_VALUE(storage::create<float>(final_path, small_options()));
    REQUIRE_VALUE(draft.save(key, raster_store::Tile<float>(4)));
    REQUIRE(REQUIRE_VALUE(io::filesystem::remove(draft.paths(key).front())));
    REQUIRE_VALUE(storage::publish(std::move(draft)));

    const storage::Storage<float> opened = REQUIRE_VALUE(storage::open<float>(final_path));
    CHECK(REQUIRE_ERROR(opened.load(key)).holds_code(io::make_file_not_found(opened.paths(key).front())));
}

TEST_CASE("raster reader selection follows metadata instead of file endings", "[raster_store][storage]")
{
    const TempDirectory directory("raster-reader-selection");
    write_table(directory.path());
    const std::filesystem::path final_path = directory.path() / "storage";
    storage::Draft<float> draft = REQUIRE_VALUE(storage::create<float>(final_path, small_options()));
    REQUIRE_VALUE(draft.save({ 0, { 0, 0 } }, raster_store::Tile<float>(4)));
    REQUIRE_VALUE(storage::publish(std::move(draft)));

    storage::Metadata metadata = REQUIRE_VALUE(storage::read_metadata(final_path));
    metadata.ids.codec_id = ".amort";
    REQUIRE_VALUE(storage::write_metadata(final_path, metadata));
    CHECK(REQUIRE_ERROR(storage::open<float>(final_path)).holds<Unsupported>());
    metadata.ids.codec_id = "amort";
    REQUIRE_VALUE(storage::write_metadata(final_path, metadata));
    REQUIRE_VALUE(storage::open<float>(final_path));
}

TEST_CASE("cross-root hard links and independent attribution survive RF removal", "[raster_store][storage]")
{
    const TempDirectory directory("raster-cross-root");
    const std::filesystem::path rf_root = directory.path() / "rf";
    const std::filesystem::path tb_root = directory.path() / "tb";
    const std::filesystem::path rf = rf_root / "storage";
    const std::filesystem::path tb = tb_root / "storage";
    write_table(rf_root);
    raster_store::Tile<float> tile(4);
    tile.data.fill(12.5f);
    {
        storage::Draft<float> created = REQUIRE_VALUE(storage::create<float>(rf, small_options()));
        REQUIRE_VALUE(created.save({ 0, { 0, 0 } }, tile));
        REQUIRE_VALUE(created.save({ 2, { 1, 1 } }, tile));
        REQUIRE_VALUE(storage::publish(std::move(created)));
    }
    {
        const storage::Storage<float> source = REQUIRE_VALUE(storage::open<float>(rf));
        storage::CreateOptions options = small_options();
        options.copy_attribution_from = rf;
        storage::Draft<float> target = REQUIRE_VALUE(storage::create<float>(tb, options));
        for (const radix::tile::Id key : { radix::tile::Id { 0, { 0, 0 } }, radix::tile::Id { 2, { 1, 1 } } }) {
            REQUIRE_VALUE(target.link_from(key, source));
            CHECK(REQUIRE_VALUE(io::filesystem::equivalent(source.paths(key).front(), target.paths(key).front())));
        }
        CHECK_FALSE(REQUIRE_VALUE(io::filesystem::equivalent(rf_root / attribution::file_name, target.base_path() / attribution::file_name)));
        CHECK(REQUIRE_VALUE(io::bytes::read(rf_root / attribution::file_name)) == REQUIRE_VALUE(io::bytes::read(target.base_path() / attribution::file_name)));
        REQUIRE_VALUE(storage::publish(std::move(target)));
    }
    REQUIRE_VALUE(io::filesystem::remove_all(rf_root));

    const storage::Storage<float> target = REQUIRE_VALUE(storage::open<float>(tb));
    CHECK(target.index().status({ 0, { 0, 0 } }) == hierarchy::NodeStatus::Inner);
    CHECK(REQUIRE_VALUE(target.load({ 2, { 1, 1 } })).data.pixel({ 0, 0 }) == 12.5f);
    REQUIRE_VALUE(attribution::read_table(tb));
}

TEST_CASE("raster opening retains metadata and index errors without creating a dataset", "[raster_store][storage]")
{
    const TempDirectory directory("raster-open-errors");
    write_table(directory.path());
    const std::filesystem::path final_path = directory.path() / "storage";
    CHECK(REQUIRE_ERROR(storage::open<float>(final_path)).holds_and<io::FileError>(is_not_found));
    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(final_path)));
    {
        storage::Draft<float> draft = REQUIRE_VALUE(storage::create<float>(final_path, small_options()));
        REQUIRE_VALUE(storage::publish(std::move(draft)));
    }

    SECTION("missing metadata")
    {
        REQUIRE(REQUIRE_VALUE(io::filesystem::remove(final_path / storage::metadata_file_name)));
        CHECK(REQUIRE_ERROR(storage::open<float>(final_path)).holds_and<io::FileError>(is_not_found));
    }
    SECTION("missing index")
    {
        REQUIRE(REQUIRE_VALUE(io::filesystem::remove(final_path / storage::index_file_name)));
        CHECK(REQUIRE_ERROR(storage::open<float>(final_path)).holds_and<io::FileError>(is_not_found));
    }
    SECTION("unknown layout")
    {
        storage::Metadata metadata = REQUIRE_VALUE(storage::read_metadata(final_path));
        metadata.ids.layout_id = "unknown";
        REQUIRE_VALUE(storage::write_metadata(final_path, metadata));
        CHECK(REQUIRE_ERROR(storage::open<float>(final_path)).holds<Unsupported>());
    }
    SECTION("rectangular metadata")
    {
        raster_store_test::v2::Metadata metadata = REQUIRE_VALUE(io::envelope::read<MetadataSchema>(final_path / storage::metadata_file_name));
        metadata.stored_tile_size = 2;
        REQUIRE_VALUE(io::envelope::write<MetadataSchema>(metadata, final_path / storage::metadata_file_name, { .mode = io::WriteMode::Overwrite }));
        CHECK(REQUIRE_ERROR(storage::open<float>(final_path)).holds<store::CorruptMetadata>());
    }
    SECTION("wrong envelope class")
    {
        REQUIRE_VALUE(io::filesystem::copy_file(final_path / storage::metadata_file_name, final_path / storage::index_file_name, io::filesystem::CopyMode::Overwrite));
        CHECK(REQUIRE_ERROR(storage::open<float>(final_path)).holds<store::CorruptIndex>());
    }
}

TEST_CASE("abandoned raster output checkpoints but never publishes", "[raster_store][storage]")
{
    const TempDirectory directory("raster-abandoned");
    write_table(directory.path());
    const std::filesystem::path final_path = directory.path() / "storage";
    const std::filesystem::path draft_path = directory.path() / "storage.part";
    const radix::tile::Id key { 0, { 0, 0 } };
    {
        storage::Draft<float> draft = REQUIRE_VALUE(storage::create<float>(final_path, small_options()));
        REQUIRE_VALUE(draft.save(key, raster_store::Tile<float>(4)));
    }
    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(final_path)));

    const storage::Storage<float> abandoned = REQUIRE_VALUE(storage::open<float>(draft_path, store::Accept::Draft));
    REQUIRE_VALUE(abandoned.load(key));
}

TEMPLATE_TEST_CASE("storage mappings and stored halos round trip", "[raster_store][storage]", float, glm::u8vec3, glm::u8vec4)
{
    const TempDirectory directory("raster-halos");
    write_table(directory.path());
    storage::CreateOptions options = small_options();
    options.halo_width = 1;
    SECTION("type default") { }
    SECTION("explicit linear") { options.value_mapping = raster_store::pixel::Mapping::Linear; }
    SECTION("explicit srgba") { options.value_mapping = raster_store::pixel::Mapping::SRGBA; }
    const std::filesystem::path final_path = directory.path() / "storage";
    const radix::tile::Id key { 0, { 0, 0 } };

    storage::Draft<TestType> draft = REQUIRE_VALUE(storage::create<TestType>(final_path, options));
    const storage::Metadata metadata = REQUIRE_VALUE(storage::read_metadata(draft.base_path()));
    CHECK(metadata.nominal_tile_size == 4);
    CHECK(metadata.stored_tile_size == 6);
    CHECK(metadata.halo_width == 1);
    CHECK(metadata.value_mapping == options.value_mapping.value_or(raster_store::pixel::default_mapping<TestType>));
    raster_store::Tile<TestType> tile(6);
    tile.data.fill(TestType { 7 });
    REQUIRE_VALUE(draft.save(key, tile));
    REQUIRE_VALUE(storage::publish(std::move(draft)));

    CHECK(REQUIRE_VALUE(storage::read_metadata(final_path)).value_mapping == metadata.value_mapping);
    const storage::Storage<TestType> opened = REQUIRE_VALUE(storage::open<TestType>(final_path));
    check_bytes(REQUIRE_VALUE(opened.load(key)).data, tile.data);
}

TEST_CASE("storage geometry and mapping validation precede creation", "[raster_store][storage]")
{
    const TempDirectory directory("raster-validation");
    write_table(directory.path());
    storage::CreateOptions options = small_options();
    SECTION("nonpower interior") { options.nominal_tile_size = 3; }
    SECTION("zero interior") { options.nominal_tile_size = 0; }
    SECTION("excessive halo") { options.halo_width = 5; }
    SECTION("overflow")
    {
        options.nominal_tile_size = 1u << 31;
        options.halo_width = 1u << 31;
    }
    SECTION("unknown mapping") { options.value_mapping = static_cast<raster_store::pixel::Mapping>(99); }
    SECTION("area overlap") { options.overlap = 1; }
    SECTION("double vertex overlap")
    {
        options.placement = raster_store::pixel::Placement::Vertex;
        options.overlap = 2;
    }
    SECTION("unknown placement") { options.placement = static_cast<raster_store::pixel::Placement>(99); }
    CHECK(REQUIRE_ERROR(storage::create<float>(directory.path() / "storage", options)).holds<storage::InvalidOptions>());
    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(directory.path() / "storage.part")));
}

TEST_CASE("raster storages checkpoint tile hashes and verify loads against them", "[raster_store][storage]")
{
    const TempDirectory directory("raster-hashes");
    write_table(directory.path());
    const std::filesystem::path final_path = directory.path() / "storage";
    const radix::tile::Id key { 0, { 0, 0 } };
    raster_store::Tile<float> tile(4);
    tile.data.fill(1.5f);
    {
        storage::Draft<float> draft = REQUIRE_VALUE(storage::create<float>(final_path, small_options()));
        REQUIRE_VALUE(draft.save(key, tile));
        CHECK(REQUIRE_VALUE(draft.hash(key)) == REQUIRE_VALUE(io::envelope::read_header(draft.paths(key).front())).hash);
        REQUIRE_VALUE(storage::publish(std::move(draft)));
    }

    const storage::Storage<float> opened = REQUIRE_VALUE(storage::open<float>(final_path));
    const std::filesystem::path path = opened.paths(key).front();
    CHECK(REQUIRE_VALUE(opened.hash(key)) == REQUIRE_VALUE(io::envelope::read_header(path)).hash);

    tile.data.fill(2.5f);
    REQUIRE(REQUIRE_VALUE(io::filesystem::remove(path)));
    const TileCodec<float> codec(4);
    REQUIRE_VALUE(codec.write(path.parent_path() / path.stem(), tile));
    CHECK(REQUIRE_ERROR(opened.load(key)).holds<store::CorruptData>());
}
