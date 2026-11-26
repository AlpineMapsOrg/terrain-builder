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

#include <catch2/catch_test_macros.hpp>
#include <glm/gtc/type_precision.hpp>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "io/WriteMode.h"
#include "io/bytes.h"
#include "io/envelope.h"
#include "raster_store/pixel.h"
#include "raster_store/storage.h"
#include "raster_store/storage/metadata.h"
#include "raster_store_test_utils.h"
#include "store/Metadata.h"

namespace {

namespace storage = raster_store::storage;
using raster_store::pixel::Mapping;
using raster_store::pixel::Placement;

// The metadata of a float storage with tiles of 4 pixels and neither halo nor overlap.
storage::Metadata small_metadata()
{
    return {
        .ids = { .layout_id = "zoom/x/y_google", .payload_class = storage::payload_class<float>, .codec_id = "amort" },
        .stored_tile_size = 4,
        .nominal_tile_size = 4,
        .halo_width = 0,
        .value_mapping = Mapping::Linear,
        .placement = Placement::Area,
        .overlap = 0,
    };
}

} // namespace

TEST_CASE("write_metadata and read_metadata round-trip both placements as version 2", "[raster_store][storage][metadata]")
{
    const TempDirectory directory("metadata-round-trip");
    storage::Metadata metadata = small_metadata();
    SECTION("area") { }
    SECTION("vertex without overlap") { metadata.placement = Placement::Vertex; }
    SECTION("vertex with overlap and halo")
    {
        metadata.placement = Placement::Vertex;
        metadata.overlap = 1;
        metadata.halo_width = 2;
        metadata.stored_tile_size = 9;
    }
    REQUIRE_VALUE(storage::write_metadata(directory.path(), metadata));
    CHECK(REQUIRE_VALUE(io::envelope::read_header(directory.path() / storage::metadata_file_name)).class_version == 2);
    CHECK(REQUIRE_VALUE(storage::read_metadata(directory.path())) == metadata);
}

TEST_CASE("read_metadata reads version 1 as area placement without overlap", "[raster_store][storage][metadata]")
{
    const TempDirectory directory("metadata-version-1");
    const raster_store_test::v1::Metadata previous { "zoom/x/y_google", storage::payload_class<glm::u8vec3>, "amoit", 6, 4, 1, Mapping::SRGBA };
    const io::bytes::vector bytes = REQUIRE_VALUE(io::envelope::serialize<raster_store_test::MetadataSchema, 1>(previous));
    REQUIRE_VALUE(io::bytes::write(bytes, directory.path() / storage::metadata_file_name, { .mode = io::WriteMode::Overwrite }));

    const storage::Metadata expected {
        .ids = { .layout_id = "zoom/x/y_google", .payload_class = storage::payload_class<glm::u8vec3>, .codec_id = "amoit" },
        .stored_tile_size = 6,
        .nominal_tile_size = 4,
        .halo_width = 1,
        .value_mapping = Mapping::SRGBA,
        .placement = Placement::Area,
        .overlap = 0,
    };
    CHECK(REQUIRE_VALUE(storage::read_metadata(directory.path())) == expected);
}

TEST_CASE("validate permits overlap only for vertex placement", "[raster_store][storage][metadata]")
{
    storage::Metadata metadata = small_metadata();
    metadata.placement = Placement::Vertex;
    metadata.overlap = 1;
    metadata.stored_tile_size = 5;
    REQUIRE_VALUE(storage::validate(metadata));
    SECTION("area overlap") { metadata.placement = Placement::Area; }
    SECTION("double overlap")
    {
        metadata.overlap = 2;
        metadata.stored_tile_size = 6;
    }
    SECTION("stored size without overlap") { metadata.stored_tile_size = 4; }
    SECTION("unknown placement") { metadata.placement = static_cast<Placement>(2); }
    CHECK(REQUIRE_ERROR(storage::validate(metadata)).holds<store::InvalidMetadata>());
}
